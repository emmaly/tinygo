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

	dma     i2sDMADesc
	dmaBuf  [i2sDMABufBytes]byte
	dmaBusy bool
}

// i2sDMABufBytes is the on-chip buffer used by a single DMA descriptor.
// It is large enough to hold one millisecond of 48 kHz stereo 16-bit
// audio plus headroom (4 * 48 = 192 bytes minimum, rounded up).
const i2sDMABufBytes = 1024

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

// Peripheral signal indices on the GPIO matrix. Values are documented
// in the ESP32 TRM Table 4-2 and ESP-IDF's gpio_sig_map.h.
const (
	gpioSigI2S0O_BCK      = 24
	gpioSigI2S0O_WS       = 25
	gpioSigI2S0O_DATA_OUT = 26 // I2S0O_DATA_OUT23, used for single-channel output
	gpioSigI2S0I_BCK      = 23
	gpioSigI2S0I_WS       = 22
	gpioSigI2S0I_DATA_IN  = 21 // I2S0I_DATA_IN0
	gpioSigI2S1O_BCK      = 31
	gpioSigI2S1O_WS       = 32
	gpioSigI2S1O_DATA_OUT = 33 // I2S1O_DATA_OUT23
	gpioSigI2S1I_BCK      = 30
	gpioSigI2S1I_WS       = 29
	gpioSigI2S1I_DATA_IN  = 28 // I2S1I_DATA_IN0
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

	// FIFO: 16-bit mono mode. tx_fifo_mod=1 selects 16-bit mono so the
	// DMA-supplied 32-bit word carries a single 16-bit sample (high
	// half ignored by hardware) and the peripheral duplicates / steers
	// onto whichever I2S slot is selected by CONF_CHAN.TX_CHAN_MOD.
	// This matches the M5Stack Atom Echo Arduino driver, which uses
	// I2S_CHANNEL_FMT_ONLY_RIGHT to feed the NS4168 amp that listens on
	// the right channel.
	i2s.Bus.SetFIFO_CONF_TX_FIFO_MOD(1)
	i2s.Bus.SetFIFO_CONF_TX_FIFO_MOD_FORCE_EN(1)
	i2s.Bus.SetFIFO_CONF_DSCR_EN(1)
	i2s.Bus.SetFIFO_CONF_TX_DATA_NUM(32)
	i2s.Bus.SetFIFO_CONF_RX_DATA_NUM(32)

	// CONF_CHAN: TX_CHAN_MOD = 1 = output mono samples on the right
	// slot of the stereo I2S frame. ESP-IDF's i2s_ll_tx_select_std_slot
	// uses the same value for non-mono I2S_STD_SLOT_RIGHT.
	i2s.Bus.SetCONF_CHAN_TX_CHAN_MOD(1)
	i2s.Bus.SetCONF_CHAN_RX_CHAN_MOD(0)
	i2s.Bus.SetSAMPLE_RATE_CONF_TX_BITS_MOD(16)
	i2s.Bus.SetSAMPLE_RATE_CONF_RX_BITS_MOD(16)

	// Clock: use 160MHz PLL_D2 source. Compute integer divider so the
	// resulting MCLK lands at SampleRate * 64 (16 bits × 2 channels ×
	// 2 BCK_DIV_NUM oversample = 64). The slight rounding error is
	// acceptable for voice-band audio.
	const mclkBase uint32 = 160_000_000
	const mclkMult uint32 = 64
	mclkTarget := config.AudioFrequency * mclkMult
	div := mclkBase / mclkTarget
	if div < 2 {
		div = 2
	}
	println("i2s: cfg.AudioFreq=", config.AudioFrequency, " mclkTarget=", mclkTarget, " div=", div)
	i2s.Bus.SetCLKM_CONF_CLK_EN(1)
	i2s.Bus.SetCLKM_CONF_CLKA_ENA(0)
	i2s.Bus.SetCLKM_CONF_CLKM_DIV_NUM(div)
	println("i2s: post DIV_NUM write, read back=", i2s.Bus.GetCLKM_CONF_CLKM_DIV_NUM())
	i2s.Bus.SetCLKM_CONF_CLKM_DIV_A(63)
	println("i2s: post DIV_A write, DIV_NUM read back=", i2s.Bus.GetCLKM_CONF_CLKM_DIV_NUM())
	i2s.Bus.SetCLKM_CONF_CLKM_DIV_B(0)
	println("i2s: post DIV_B write, DIV_NUM read back=", i2s.Bus.GetCLKM_CONF_CLKM_DIV_NUM())
	i2s.Bus.SetSAMPLE_RATE_CONF_TX_BCK_DIV_NUM(2)
	i2s.Bus.SetSAMPLE_RATE_CONF_RX_BCK_DIV_NUM(2)

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

	// Prep DMA descriptor as self-linked single buffer.
	i2s.dma.buf = &i2s.dmaBuf[0]
	i2s.dma.next = &i2s.dma
	i2s.dma.flags = uint32(i2sDMABufBytes) | (uint32(i2sDMABufBytes) << 12) | i2sDescEOF | i2sDescOwnerCPU

	return nil
}

// SetSampleFrequency updates the sample rate. Configure must have been
// called first.
func (i2s *I2S) SetSampleFrequency(freq uint32) error {
	if !i2sFreqSupported(freq) {
		return errI2SBadFreq
	}
	i2s.conf.AudioFrequency = freq
	const mclkBase uint32 = 160_000_000
	const mclkMult uint32 = 64
	mclkTarget := freq * mclkMult
	div := mclkBase / mclkTarget
	if div < 2 {
		div = 2
	}
	i2s.Bus.SetCLKM_CONF_CLKM_DIV_NUM(div)
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
// on the configured TX pins. The same sample is duplicated on both L
// and R slots of the I2S frame.
func (i2s *I2S) WriteMono(b []uint16) (int, error) {
	if i2s.dmaBusy {
		return 0, errI2SBusy
	}
	written := 0
	for written < len(b) {
		// Each DMA buffer slot holds a 32-bit stereo frame: low 16
		// bits = right, high 16 bits = left. For mono output we put
		// the same value in both halves.
		slot := i2s.dmaBuf[:]
		samplesThisRound := (i2sDMABufBytes / 4)
		if samplesThisRound > len(b)-written {
			samplesThisRound = len(b) - written
		}
		// FIFO_MOD=1 (16-bit mono) consumes one 16-bit sample per
		// 32-bit DMA word; the upper 16 bits are ignored, but we
		// duplicate them so a future switch to FIFO_MOD=0 dual
		// channel doesn't change the audio.
		for i := 0; i < samplesThisRound; i++ {
			s := uint32(b[written+i])
			frame := (s << 16) | s
			slot[i*4+0] = byte(frame)
			slot[i*4+1] = byte(frame >> 8)
			slot[i*4+2] = byte(frame >> 16)
			slot[i*4+3] = byte(frame >> 24)
		}
		bytesThisRound := uint32(samplesThisRound * 4)
		i2s.dma.flags = bytesThisRound | (bytesThisRound << 12) | i2sDescEOF | i2sDescOwnerDMA
		i2s.armOutLink()
		i2s.Bus.SetCONF_TX_START(1)
		if err := i2s.waitTXEOF(); err != nil {
			return written, err
		}
		written += samplesThisRound
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
	written := 0
	for written < len(b) {
		samplesThisRound := (i2sDMABufBytes / 4)
		if samplesThisRound > len(b)-written {
			samplesThisRound = len(b) - written
		}
		for i := 0; i < samplesThisRound; i++ {
			frame := b[written+i]
			i2s.dmaBuf[i*4+0] = byte(frame)
			i2s.dmaBuf[i*4+1] = byte(frame >> 8)
			i2s.dmaBuf[i*4+2] = byte(frame >> 16)
			i2s.dmaBuf[i*4+3] = byte(frame >> 24)
		}
		bytesThisRound := uint32(samplesThisRound * 4)
		i2s.dma.flags = bytesThisRound | (bytesThisRound << 12) | i2sDescEOF | i2sDescOwnerDMA
		i2s.armOutLink()
		i2s.Bus.SetCONF_TX_START(1)
		if err := i2s.waitTXEOF(); err != nil {
			return written, err
		}
		written += samplesThisRound
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
	// Match the canonical ESP-IDF tx-channel start sequence: reset TX
	// path + FIFO, re-enable the OUT_EOF interrupt status path, then
	// program OUT_LINK.addr | start in one combined write. Splitting
	// addr/start writes can leave OUT_LINK inconsistent (DMA flagged
	// OUT_DSCR_ERR during early bring-up); writing the field-bundle
	// together avoids it.
	i2s.Bus.SetCONF_TX_RESET(1)
	i2s.Bus.SetCONF_TX_RESET(0)
	i2s.Bus.SetLC_CONF_OUT_RST(1)
	i2s.Bus.SetLC_CONF_OUT_RST(0)
	i2s.Bus.SetCONF_TX_FIFO_RESET(1)
	i2s.Bus.SetCONF_TX_FIFO_RESET(0)
	i2s.Bus.SetINT_ENA_OUT_EOF_INT_ENA(1)
	i2s.Bus.SetFIFO_CONF_DSCR_EN(1)
	addr := uint32(uintptr(unsafe.Pointer(&i2s.dma))) & 0xfffff
	const outLinkStart = uint32(1 << 29)
	i2s.Bus.OUT_LINK.Set(addr | outLinkStart)
	i2s.Bus.SetINT_CLR_OUT_EOF_INT_CLR(1)
	i2s.Bus.SetINT_CLR_OUT_DSCR_ERR_INT_CLR(1)
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

