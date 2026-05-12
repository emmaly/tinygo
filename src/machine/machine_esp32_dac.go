//go:build esp32

package machine

import "device/esp"

// DAC on the ESP32.
//
// The chip exposes two independent 8-bit DAC channels. Channel 0 (the
// chip's "DAC1") drives GPIO25 and channel 1 (the chip's "DAC2") drives
// GPIO26. Each channel must be enabled before use; the matching pin
// should not also be configured as a digital GPIO.
type DAC struct {
	Channel uint8
}

var (
	DAC0 = DAC{Channel: 0}
	DAC1 = DAC{Channel: 1}
)

// DACConfig placeholder for future expansion.
type DACConfig struct {
}

// Configure powers up the DAC channel and routes its output pad.
func (dac DAC) Configure(config DACConfig) {
	// Output value comes from the per-channel PDACn_DAC field, not the
	// digital subsystem cosine generator.
	esp.SENS.SetSAR_DAC_CTRL1_DAC_DIG_FORCE(0)

	switch dac.Channel {
	case 0:
		esp.SENS.SetSAR_DAC_CTRL2_DAC_CW_EN1(0)
		esp.RTC_IO.SetPAD_DAC1_PDAC1_MUX_SEL(1)
		esp.RTC_IO.SetPAD_DAC1_PDAC1_FUN_IE(0)
		esp.RTC_IO.SetPAD_DAC1_PDAC1_RUE(0)
		esp.RTC_IO.SetPAD_DAC1_PDAC1_RDE(0)
		esp.RTC_IO.SetPAD_DAC1_PDAC1_XPD_DAC(1)
		esp.RTC_IO.SetPAD_DAC1_PDAC1_DAC_XPD_FORCE(1)
	default:
		esp.SENS.SetSAR_DAC_CTRL2_DAC_CW_EN2(0)
		esp.RTC_IO.SetPAD_DAC2_PDAC2_MUX_SEL(1)
		esp.RTC_IO.SetPAD_DAC2_PDAC2_FUN_IE(0)
		esp.RTC_IO.SetPAD_DAC2_PDAC2_RUE(0)
		esp.RTC_IO.SetPAD_DAC2_PDAC2_RDE(0)
		esp.RTC_IO.SetPAD_DAC2_PDAC2_XPD_DAC(1)
		esp.RTC_IO.SetPAD_DAC2_PDAC2_DAC_XPD_FORCE(1)
	}
}

// Set writes a single 16-bit value to the DAC.
// Since the ESP32 only has an 8-bit DAC, the high 8 bits of the passed-in
// value are used.
func (dac DAC) Set(value uint16) error {
	v := uint32(value >> 8)
	switch dac.Channel {
	case 0:
		esp.RTC_IO.SetPAD_DAC1_PDAC1_DAC(v)
	default:
		esp.RTC_IO.SetPAD_DAC2_PDAC2_DAC(v)
	}
	return nil
}
