//go:build esp32

package machine

import (
	"device/esp"
	"errors"
	"unsafe"
)

// I2S on the ESP32.
//
// The chip has two independent I2S controllers, I2S0 and I2S1. This
// driver currently supports master-mode TX (output) using the standard
// Philips I2S framing. Output samples are streamed through the I2S DMA
// engine using a single self-linked descriptor that the driver re-arms
// before each transfer. PDM input on I2S0 is planned but not yet wired
// up.
//
// The ESP32 routes every I2S signal through the IO matrix, so any free
// GPIO can be used as BCK, WS or DATA. Configure() programs the matrix
// to drive the requested pins.

type I2S struct {
	Bus  *esp.I2S_Type
	id   uint8 // 0 = I2S0, 1 = I2S1
	conf I2SConfig

	dma       [i2sDMADescCount]i2sDMADesc
	dmaBuf    [i2sDMADescCount][i2sDMABufBytes]byte
	dmaIdx    int // next descriptor index to fill
	dmaBusy   bool
	dmaPrimed bool
}

// I2S DMA ring: a fixed number of self-linked descriptors that the
// DMA controller cycles through continuously. With a single-descriptor
// arrangement the peripheral FIFO drains during the time between
// WriteMono calls and the NS4168 amp on Atom Echo interprets the
// resulting silence as "stop" and ramps its output down, which kills
// the perceived volume of subsequent buffers. A 4-descriptor ring
// keeps the FIFO continuously fed while the CPU is refilling the
// descriptor that was just emptied.
const (
	i2sDMADescCount = 4
	i2sDMABufBytes  = 512
)

// I2S DMA descriptor as expected by the ESP32 DMA controller. Layout
// and field placement is fixed by hardware; do not reorder.
type i2sDMADesc struct {
	flags uint32 // [11..0] size, [23..12] length, [27..24] reserved, [28] sosf, [29] eof, [30] owner, [31] reserved
	buf   *byte
	next  *i2sDMADesc
}

// DMA descriptor flag layout (lldesc_t):
//
//	[11:0]  size   buffer capacity in bytes
//	[23:12] length valid data length in bytes
//	[28:24] offset reserved (used by software for sub-buffer offsets)
//	[29]    sosf   start of sub-frame
//	[30]    eof    end of frame
//	[31]    owner  1 = hardware owns descriptor, 0 = software
const (
	i2sDescOwnerCPU uint32 = 0 << 31
	i2sDescOwnerDMA uint32 = 1 << 31
	i2sDescEOF      uint32 = 1 << 30
)

// Peripheral signal indices on the GPIO matrix, per ESP-IDF's
// components/soc/esp32/include/soc/gpio_sig_map.h.
//
// Master TX uses the I2S0O_* / I2S1O_* index family: the peripheral
// drives BCK, WS and DATA out on these signal numbers. Master RX uses
// the I2S0I_* / I2S1I_* family for DATA in (and may drive its own BCK
// and WS for slave-clock-out style configurations).
const (
	gpioSigI2S0O_BCK      = 23
	gpioSigI2S0O_WS       = 25
	gpioSigI2S0O_DATA_OUT = 163 // I2S0O_DATA_OUT23, single-channel data
	gpioSigI2S0I_DATA_IN  = 140 // I2S0I_DATA_IN0
	gpioSigI2S1O_BCK      = 24
	gpioSigI2S1O_WS       = 26
	gpioSigI2S1O_DATA_OUT = 189 // I2S1O_DATA_OUT23
	gpioSigI2S1I_DATA_IN  = 166 // I2S1I_DATA_IN0
)

var (
	I2S0 = I2S{Bus: esp.I2S0, id: 0}
	I2S1 = I2S{Bus: esp.I2S1, id: 1}
)

var (
	errI2SBadFreq   = ErrInvalidSampleFrequency
	errI2SBadConfig = errors.New("i2s: invalid configuration")
	errI2SBusy      = errors.New("i2s: transfer already in progress")
)

// Configure prepares the I2S controller for transfer. The driver
// configures master TX in the Philips standard with 16-bit samples.
// SCK, WS and SDO from config are routed through the IO matrix. Any
// other I2S parameters in the config are accepted but currently
// ignored.
func (i2s *I2S) Configure(config I2SConfig) error {
	if config.AudioFrequency == 0 {
		config.AudioFrequency = 16000
	}
	if !i2sFreqSupported(config.AudioFrequency) {
		return errI2SBadFreq
	}
	i2s.conf = config

	i2sEnablePeripheralClock(i2s.id)

	// Reset RX and TX paths, FIFOs, and DMA.
	i2s.Bus.SetCONF_TX_RESET(1)
	i2s.Bus.SetCONF_TX_FIFO_RESET(1)
	i2s.Bus.SetCONF_RX_RESET(1)
	i2s.Bus.SetCONF_RX_FIFO_RESET(1)
	i2s.Bus.SetLC_CONF_IN_RST(1)
	i2s.Bus.SetLC_CONF_OUT_RST(1)
	i2s.Bus.SetLC_CONF_AHBM_FIFO_RST(1)
	i2s.Bus.SetLC_CONF_AHBM_RST(1)
	i2s.Bus.SetCONF_TX_RESET(0)
	i2s.Bus.SetCONF_TX_FIFO_RESET(0)
	i2s.Bus.SetCONF_RX_RESET(0)
	i2s.Bus.SetCONF_RX_FIFO_RESET(0)
	i2s.Bus.SetLC_CONF_IN_RST(0)
	i2s.Bus.SetLC_CONF_OUT_RST(0)
	i2s.Bus.SetLC_CONF_AHBM_FIFO_RST(0)
	i2s.Bus.SetLC_CONF_AHBM_RST(0)

	// DMA controller config: enable burst modes so DMA keeps the
	// FIFO ahead of the audio bit clock, and have it respect the
	// per-descriptor OWNER bit.
	i2s.Bus.SetLC_CONF_OUT_DATA_BURST_EN(1)
	i2s.Bus.SetLC_CONF_OUTDSCR_BURST_EN(1)
	i2s.Bus.SetLC_CONF_INDSCR_BURST_EN(1)
	i2s.Bus.SetLC_CONF_CHECK_OWNER(1)
	i2s.Bus.SetLC_CONF_MEM_TRANS_EN(0)

	// Master mode on the TX path.
	i2s.Bus.SetCONF_TX_SLAVE_MOD(0)
	// Philips standard: shift MSB out, one BCK delay before first bit.
	i2s.Bus.SetCONF_TX_MSB_SHIFT(1)
	// Left channel first.
	i2s.Bus.SetCONF_TX_RIGHT_FIRST(0)
	// Stereo frame (LR pair) on the wire; mono callers get the same
	// sample placed on both slots.
	i2s.Bus.SetCONF_TX_MONO(0)
	i2s.Bus.SetCONF_TX_SHORT_SYNC(0)
	i2s.Bus.SetCONF_SIG_LOOPBACK(0)
	// Bypass the PCM compander. With this clear the chip pushes
	// samples through an A-law/μ-law path that mangles linear PCM.
	i2s.Bus.SetCONF1_TX_PCM_BYPASS(1)
	i2s.Bus.SetCONF1_RX_PCM_BYPASS(1)
	// Do not auto-stop the TX path at end-of-frame; we want continuous
	// streaming as long as descriptors are queued.
	i2s.Bus.SetCONF1_TX_STOP_EN(0)

	// Force into standard I2S framing. After reset CONF2 and PDM_CONF
	// may carry stale bits that route the TX path through the parallel
	// LCD or PDM data paths; without explicitly clearing them the FIFO
	// is wired to the wrong output. This matches ESP-IDF's
	// i2s_ll_tx_enable_std / i2s_ll_rx_enable_std for the ESP32.
	i2s.Bus.CONF2.Set(0)
	i2s.Bus.SetPDM_CONF_TX_PDM_EN(0)
	i2s.Bus.SetPDM_CONF_PCM2PDM_CONV_EN(0)
	i2s.Bus.SetPDM_CONF_RX_PDM_EN(0)
	i2s.Bus.SetPDM_CONF_PDM2PCM_CONV_EN(0)

	// FIFO: try 16-bit dual-channel (TX_FIFO_MOD=0) so each DMA word
	// carries an [L | R] frame. Mono single-slot (FIFO_MOD=1) appeared
	// to push the peripheral through frames 8× faster than expected.
	i2s.Bus.SetFIFO_CONF_TX_FIFO_MOD(0)
	i2s.Bus.SetFIFO_CONF_TX_FIFO_MOD_FORCE_EN(1)
	i2s.Bus.SetFIFO_CONF_DSCR_EN(1)
	i2s.Bus.SetFIFO_CONF_TX_DATA_NUM(32)
	i2s.Bus.SetFIFO_CONF_RX_DATA_NUM(32)

	// CONF_CHAN: TX_CHAN_MOD=0 = dual-channel stereo (matches
	// FIFO_MOD=0 above; both halves of each DMA word land on the
	// wire).
	i2s.Bus.SetCONF_CHAN_TX_CHAN_MOD(0)
	i2s.Bus.SetCONF_CHAN_RX_CHAN_MOD(0)
	i2s.Bus.SetSAMPLE_RATE_CONF_TX_BITS_MOD(16)
	i2s.Bus.SetSAMPLE_RATE_CONF_RX_BITS_MOD(16)

	// Clock: use 160MHz PLL_D2 source. Compute integer divider so the
	// resulting MCLK lands at SampleRate * 64 (16 bits × 2 channels ×
	// 2 BCK_DIV_NUM oversample = 64). The slight rounding error is
	// acceptable for voice-band audio.
	// Clock formula from ESP-IDF i2s_std.c i2s_std_calculate_clock:
	//
	//   bclk  = sample_rate × total_slot × slot_bits
	//   mclk  = sample_rate × mclk_multiple
	//   bclk_div = mclk / bclk
	//   mclk_div = sclk / mclk    (the ESP32 CLKM_DIV_NUM field)
	//
	// Both slots are always clocked through (the unused mono slot is
	// not trimmed), so total_slot = 2 and slot_bits = 16 for the
	// 16-bit mono / stereo settings used here. mclk_multiple = 256 is
	// a standard audio oversample ratio. For sample_rate = 16 kHz
	// against the 160 MHz PLL_F160M source this lands DIV_NUM = 39
	// and TX_BCK_DIV_NUM = 8 (the peripheral rejects bclk_div < 8).
	const mclkBase uint32 = 160_000_000
	const mclkMultiple uint32 = 256
	const totalSlot uint32 = 2
	const slotBits uint32 = 16
	mclk := config.AudioFrequency * mclkMultiple
	bclk := config.AudioFrequency * totalSlot * slotBits
	bckDiv := mclk / bclk
	if bckDiv < 8 {
		bckDiv = 8
	}
	div := mclkBase / mclk
	if div < 2 {
		div = 2
	}
	i2s.Bus.SetCLKM_CONF_CLK_EN(1)
	i2s.Bus.SetCLKM_CONF_CLKA_ENA(0)
	i2s.Bus.SetCLKM_CONF_CLKM_DIV_NUM(div)
	i2s.Bus.SetCLKM_CONF_CLKM_DIV_A(1)
	i2s.Bus.SetCLKM_CONF_CLKM_DIV_B(0)
	i2s.Bus.SetSAMPLE_RATE_CONF_TX_BCK_DIV_NUM(bckDiv)
	i2s.Bus.SetSAMPLE_RATE_CONF_RX_BCK_DIV_NUM(bckDiv)

	// Route signals through IO matrix using the existing Pin.configure
	// helper, which sets the IO_MUX function to GPIO, enables the
	// output, and writes the peripheral signal index to the per-pin
	// FUNCx_OUT_SEL_CFG register in one step.
	bckSig, wsSig, dataSig := i2s.txSignals()
	if config.SCK != NoPin {
		config.SCK.configure(PinConfig{Mode: PinOutput}, bckSig)
	}
	if config.WS != NoPin {
		config.WS.configure(PinConfig{Mode: PinOutput}, wsSig)
	}
	if config.SDO != NoPin {
		config.SDO.configure(PinConfig{Mode: PinOutput}, dataSig)
	}

	// Prep DMA descriptor ring. Each descriptor points at its own
	// buffer and links to the next; the last descriptor links back
	// to the first, so the DMA controller cycles through them
	// indefinitely. All start in software-owned state with the
	// full buffer pre-zeroed so the first arm sends silence rather
	// than uninitialised memory.
	for i := range i2s.dma {
		i2s.dma[i].buf = &i2s.dmaBuf[i][0]
		next := (i + 1) % i2sDMADescCount
		i2s.dma[i].next = &i2s.dma[next]
		i2s.dma[i].flags = uint32(i2sDMABufBytes) | (uint32(i2sDMABufBytes) << 12) | i2sDescEOF | i2sDescOwnerDMA
		for b := range i2s.dmaBuf[i] {
			i2s.dmaBuf[i][b] = 0
		}
	}
	i2s.dmaIdx = 0
	i2s.dmaPrimed = false

	return nil
}

// SetSampleFrequency updates the sample rate. Configure must have been
// called first; the bit clock divider and framing settled by Configure
// are reused, so the supported range is the same.
func (i2s *I2S) SetSampleFrequency(freq uint32) error {
	if !i2sFreqSupported(freq) {
		return errI2SBadFreq
	}
	i2s.conf.AudioFrequency = freq
	const mclkBase uint32 = 160_000_000
	const mclkMultiple uint32 = 256
	const totalSlot uint32 = 2
	const slotBits uint32 = 16
	mclk := freq * mclkMultiple
	bclk := freq * totalSlot * slotBits
	bckDiv := mclk / bclk
	if bckDiv < 8 {
		bckDiv = 8
	}
	div := mclkBase / mclk
	if div < 2 {
		div = 2
	}
	i2s.Bus.SetCLKM_CONF_CLKM_DIV_NUM(div)
	i2s.Bus.SetSAMPLE_RATE_CONF_TX_BCK_DIV_NUM(bckDiv)
	i2s.Bus.SetSAMPLE_RATE_CONF_RX_BCK_DIV_NUM(bckDiv)
	return nil
}

// Enable starts or stops the TX path.
func (i2s *I2S) Enable(enabled bool) {
	if enabled {
		i2s.Bus.SetCONF_TX_START(1)
	} else {
		i2s.Bus.SetCONF_TX_START(0)
		// Clear FIFO to drop pending samples.
		i2s.Bus.SetCONF_TX_FIFO_RESET(1)
		i2s.Bus.SetCONF_TX_FIFO_RESET(0)
	}
}

// WriteMono blocks until len(b) 16-bit samples have been streamed out
// on the configured TX pins. The mono sample is duplicated into both
// slots of each stereo frame.
//
// The DMA controller continuously cycles through the descriptor ring
// initialised in Configure. Each iteration here waits for the next
// OUT_EOF interrupt (DMA finished a buffer), then overwrites the
// buffer DMA just finished with the next chunk of samples and hands
// the descriptor back to DMA. As long as the CPU is faster than the
// audio bit rate (true at any reasonable sample rate on the ESP32)
// the peripheral never sees a silent gap.
func (i2s *I2S) WriteMono(b []uint16) (int, error) {
	if i2s.dmaBusy {
		return 0, errI2SBusy
	}
	const samplesPerDesc = i2sDMABufBytes / 4
	if !i2s.dmaPrimed {
		i2s.armOutLink()
		i2s.Bus.SetCONF_TX_START(1)
	}
	written := 0
	for written < len(b) {
		if err := i2s.waitTXEOF(); err != nil {
			return written, err
		}
		samplesThisRound := samplesPerDesc
		if samplesThisRound > len(b)-written {
			samplesThisRound = len(b) - written
		}
		slot := &i2s.dmaBuf[i2s.dmaIdx]
		for i := 0; i < samplesThisRound; i++ {
			s := uint32(b[written+i])
			frame := (s << 16) | s
			slot[i*4+0] = byte(frame)
			slot[i*4+1] = byte(frame >> 8)
			slot[i*4+2] = byte(frame >> 16)
			slot[i*4+3] = byte(frame >> 24)
		}
		bytesThisRound := uint32(samplesThisRound * 4)
		// length field tells the DMA controller how many bytes to
		// stream out before raising OUT_EOF; capacity stays the full
		// buffer size so the descriptor is reusable for larger
		// chunks. With length<size the descriptor never advances
		// past the valid data, so unused tail bytes are not
		// transmitted as silence.
		i2s.dma[i2s.dmaIdx].flags = uint32(i2sDMABufBytes) | (bytesThisRound << 12) | i2sDescEOF | i2sDescOwnerDMA
		written += samplesThisRound
		i2s.dmaIdx = (i2s.dmaIdx + 1) % i2sDMADescCount
	}
	return written, nil
}

// WriteStereo blocks until len(b) 32-bit stereo samples have been
// streamed. Each sample packs left in the high 16 bits and right in
// the low 16 bits.
func (i2s *I2S) WriteStereo(b []uint32) (int, error) {
	if i2s.dmaBusy {
		return 0, errI2SBusy
	}
	const samplesPerDesc = i2sDMABufBytes / 4
	if !i2s.dmaPrimed {
		i2s.armOutLink()
		i2s.Bus.SetCONF_TX_START(1)
	}
	written := 0
	for written < len(b) {
		if err := i2s.waitTXEOF(); err != nil {
			return written, err
		}
		samplesThisRound := samplesPerDesc
		if samplesThisRound > len(b)-written {
			samplesThisRound = len(b) - written
		}
		slot := &i2s.dmaBuf[i2s.dmaIdx]
		for i := 0; i < samplesThisRound; i++ {
			frame := b[written+i]
			slot[i*4+0] = byte(frame)
			slot[i*4+1] = byte(frame >> 8)
			slot[i*4+2] = byte(frame >> 16)
			slot[i*4+3] = byte(frame >> 24)
		}
		bytesThisRound := uint32(samplesThisRound * 4)
		// length field tells the DMA controller how many bytes to
		// stream out before raising OUT_EOF; capacity stays the full
		// buffer size so the descriptor is reusable for larger
		// chunks. With length<size the descriptor never advances
		// past the valid data, so unused tail bytes are not
		// transmitted as silence.
		i2s.dma[i2s.dmaIdx].flags = uint32(i2sDMABufBytes) | (bytesThisRound << 12) | i2sDescEOF | i2sDescOwnerDMA
		written += samplesThisRound
		i2s.dmaIdx = (i2s.dmaIdx + 1) % i2sDMADescCount
	}
	return written, nil
}

// ReadMono is not yet implemented for ESP32 I2S.
func (i2s *I2S) ReadMono(b []uint16) (int, error) {
	return 0, errors.New("i2s: ReadMono not implemented on ESP32 yet")
}

// ReadStereo is not yet implemented for ESP32 I2S.
func (i2s *I2S) ReadStereo(b []uint32) (int, error) {
	return 0, errors.New("i2s: ReadStereo not implemented on ESP32 yet")
}

func (i2s *I2S) txSignals() (bck, ws, data uint32) {
	if i2s.id == 0 {
		return gpioSigI2S0O_BCK, gpioSigI2S0O_WS, gpioSigI2S0O_DATA_OUT
	}
	return gpioSigI2S1O_BCK, gpioSigI2S1O_WS, gpioSigI2S1O_DATA_OUT
}

func (i2s *I2S) armOutLink() {
	// One-shot arm at the head of the ring. After this the DMA
	// controller follows the next-pointers continuously; the CPU
	// only refills descriptors as they emit OUT_EOF.
	i2s.Bus.SetCONF_TX_RESET(1)
	i2s.Bus.SetCONF_TX_RESET(0)
	i2s.Bus.SetLC_CONF_OUT_RST(1)
	i2s.Bus.SetLC_CONF_OUT_RST(0)
	i2s.Bus.SetCONF_TX_FIFO_RESET(1)
	i2s.Bus.SetCONF_TX_FIFO_RESET(0)
	i2s.Bus.SetINT_ENA_OUT_EOF_INT_ENA(1)
	i2s.Bus.SetFIFO_CONF_DSCR_EN(1)
	addr := uint32(uintptr(unsafe.Pointer(&i2s.dma[0]))) & 0xfffff
	const outLinkStart = uint32(1 << 29)
	i2s.Bus.OUT_LINK.Set(addr | outLinkStart)
	i2s.Bus.SetINT_CLR_OUT_EOF_INT_CLR(1)
	i2s.Bus.SetINT_CLR_OUT_DSCR_ERR_INT_CLR(1)
	i2s.dmaPrimed = true
}

func (i2s *I2S) waitTXEOF() error {
	for {
		if i2s.Bus.GetINT_RAW_OUT_EOF_INT_RAW() != 0 {
			i2s.Bus.SetINT_CLR_OUT_EOF_INT_CLR(1)
			return nil
		}
	}
}

func i2sFreqSupported(freq uint32) bool {
	return freq >= 4000 && freq <= 96000
}

// i2sEnablePeripheralClock toggles the per-controller clock and reset
// bits in DPORT. After boot these bits are typically unset; the I2S
// peripheral does not respond to register writes until they are
// enabled.
func i2sEnablePeripheralClock(id uint8) {
	if id == 0 {
		esp.DPORT.SetPERIP_CLK_EN_I2S0_CLK_EN(1)
		esp.DPORT.SetPERIP_RST_EN_I2S0_RST(1)
		esp.DPORT.SetPERIP_RST_EN_I2S0_RST(0)
	} else {
		esp.DPORT.SetPERIP_CLK_EN_I2S1_CLK_EN(1)
		esp.DPORT.SetPERIP_RST_EN_I2S1_RST(1)
		esp.DPORT.SetPERIP_RST_EN_I2S1_RST(0)
	}
}

