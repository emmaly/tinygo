//go:build m5atom_echo

// This file contains the pin mapping for the M5Stack ATOM Echo device.
// doc: https://docs.m5stack.com/en/core/atom_echo
//
// The ATOM Echo is built around the ESP32-PICO-D4 SoC. In addition to the
// shared ATOM chassis (button, status RGB LED, IR LED, Grove port), the
// Echo carries an SPM1423 PDM microphone and an NS4168 I2S amplifier
// driving an internal speaker. The PDM clock and the I2S word-select line
// share GPIO33; the board mode (record vs. playback) is selected in
// firmware by reconfiguring the pin function.

package machine

const (
	IO0  = GPIO0
	IO1  = GPIO1  // U0TXD
	IO3  = GPIO3  // U0RXD
	IO19 = GPIO19 // I2S BCK (speaker)
	IO21 = GPIO21
	IO22 = GPIO22 // I2S DATA (speaker)
	IO23 = GPIO23 // PDM DATA (microphone)
	IO25 = GPIO25
	IO26 = GPIO26 // Grove SCL
	IO27 = GPIO27 // status WS2812 RGB LED (single)
	IO32 = GPIO32 // Grove SDA
	IO33 = GPIO33 // PDM CLK (microphone) / I2S LRCK (speaker) — shared
	IO39 = GPIO39 // BUTTON
)

// Buttons
const (
	BUTTON_A = IO39
	BUTTON   = BUTTON_A
)

// Onboard peripherals
const (
	// Single WS2812 status RGB LED.
	WS2812 = IO27
	LED    = WS2812
)

// I2S / PDM pins for the SPM1423 microphone and NS4168 speaker amplifier.
//
// I2S0 is shared between the microphone (RX, PDM mode) and the speaker (TX,
// standard I2S). Reconfigure the peripheral between RX and TX before use;
// simultaneous record + playback is not supported on this hardware.
const (
	// SPM1423 PDM microphone.
	MIC_PDM_CLK = IO33
	MIC_PDM_DAT = IO23

	// NS4168 I2S speaker amplifier.
	SPK_I2S_BCK  = IO19
	SPK_I2S_LRCK = IO33
	SPK_I2S_DATA = IO22
)

// UART pins
const (
	// UART0 (USB-to-UART bridge: FT232 / CP2104 / CH9102 depending on board revision).
	UART0_TX_PIN = IO1
	UART0_RX_PIN = IO3

	UART_TX_PIN = UART0_TX_PIN
	UART_RX_PIN = UART0_RX_PIN
)

// I2C pins (Grove port on the bottom of the device).
const (
	SDA_PIN = IO32
	SCL_PIN = IO26
)
