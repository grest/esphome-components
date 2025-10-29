#pragma once

#include <cstdint>

namespace esphome {
namespace wmbus_radio {

// CC1101 configuration registers
inline constexpr uint8_t CC1101_IOCFG2 = 0x00;
inline constexpr uint8_t CC1101_IOCFG1 = 0x01;
inline constexpr uint8_t CC1101_IOCFG0 = 0x02;
inline constexpr uint8_t CC1101_FIFOTHR = 0x03;
inline constexpr uint8_t CC1101_SYNC1 = 0x04;
inline constexpr uint8_t CC1101_SYNC0 = 0x05;
inline constexpr uint8_t CC1101_PKTLEN = 0x06;
inline constexpr uint8_t CC1101_PKTCTRL1 = 0x07;
inline constexpr uint8_t CC1101_PKTCTRL0 = 0x08;
inline constexpr uint8_t CC1101_ADDR = 0x09;
inline constexpr uint8_t CC1101_CHANNR = 0x0A;
inline constexpr uint8_t CC1101_FSCTRL1 = 0x0B;
inline constexpr uint8_t CC1101_FSCTRL0 = 0x0C;
inline constexpr uint8_t CC1101_FREQ2 = 0x0D;
inline constexpr uint8_t CC1101_FREQ1 = 0x0E;
inline constexpr uint8_t CC1101_FREQ0 = 0x0F;
inline constexpr uint8_t CC1101_MDMCFG4 = 0x10;
inline constexpr uint8_t CC1101_MDMCFG3 = 0x11;
inline constexpr uint8_t CC1101_MDMCFG2 = 0x12;
inline constexpr uint8_t CC1101_MDMCFG1 = 0x13;
inline constexpr uint8_t CC1101_MDMCFG0 = 0x14;
inline constexpr uint8_t CC1101_DEVIATN = 0x15;
inline constexpr uint8_t CC1101_MCSM2 = 0x16;
inline constexpr uint8_t CC1101_MCSM1 = 0x17;
inline constexpr uint8_t CC1101_MCSM0 = 0x18;
inline constexpr uint8_t CC1101_FOCCFG = 0x19;
inline constexpr uint8_t CC1101_BSCFG = 0x1A;
inline constexpr uint8_t CC1101_AGCCTRL2 = 0x1B;
inline constexpr uint8_t CC1101_AGCCTRL1 = 0x1C;
inline constexpr uint8_t CC1101_AGCCTRL0 = 0x1D;
inline constexpr uint8_t CC1101_WOREVT1 = 0x1E;
inline constexpr uint8_t CC1101_WOREVT0 = 0x1F;
inline constexpr uint8_t CC1101_WORCTRL = 0x20;
inline constexpr uint8_t CC1101_FREND1 = 0x21;
inline constexpr uint8_t CC1101_FREND0 = 0x22;
inline constexpr uint8_t CC1101_FSCAL3 = 0x23;
inline constexpr uint8_t CC1101_FSCAL2 = 0x24;
inline constexpr uint8_t CC1101_FSCAL1 = 0x25;
inline constexpr uint8_t CC1101_FSCAL0 = 0x26;
inline constexpr uint8_t CC1101_RCCTRL1 = 0x27;
inline constexpr uint8_t CC1101_RCCTRL0 = 0x28;
inline constexpr uint8_t CC1101_FSTEST = 0x29;
inline constexpr uint8_t CC1101_PTEST = 0x2A;
inline constexpr uint8_t CC1101_AGCTEST = 0x2B;
inline constexpr uint8_t CC1101_TEST2 = 0x2C;
inline constexpr uint8_t CC1101_TEST1 = 0x2D;
inline constexpr uint8_t CC1101_TEST0 = 0x2E;

// CC1101 strobe commands
inline constexpr uint8_t CC1101_SRES = 0x30;
inline constexpr uint8_t CC1101_SFSTXON = 0x31;
inline constexpr uint8_t CC1101_SXOFF = 0x32;
inline constexpr uint8_t CC1101_SCAL = 0x33;
inline constexpr uint8_t CC1101_SRX = 0x34;
inline constexpr uint8_t CC1101_STX = 0x35;
inline constexpr uint8_t CC1101_SIDLE = 0x36;
inline constexpr uint8_t CC1101_SAFC = 0x37;
inline constexpr uint8_t CC1101_SWOR = 0x38;
inline constexpr uint8_t CC1101_SPWD = 0x39;
inline constexpr uint8_t CC1101_SFRX = 0x3A;
inline constexpr uint8_t CC1101_SFTX = 0x3B;
inline constexpr uint8_t CC1101_SWORRST = 0x3C;
inline constexpr uint8_t CC1101_SNOP = 0x3D;

// CC1101 status registers
inline constexpr uint8_t CC1101_PARTNUM = 0x30;
inline constexpr uint8_t CC1101_VERSION = 0x31;
inline constexpr uint8_t CC1101_FREQEST = 0x32;
inline constexpr uint8_t CC1101_LQI = 0x33;
inline constexpr uint8_t CC1101_RSSI = 0x34;
inline constexpr uint8_t CC1101_MARCSTATE = 0x35;
inline constexpr uint8_t CC1101_WORTIME1 = 0x36;
inline constexpr uint8_t CC1101_WORTIME0 = 0x37;
inline constexpr uint8_t CC1101_PKTSTATUS = 0x38;
inline constexpr uint8_t CC1101_VCO_VC_DAC = 0x39;
inline constexpr uint8_t CC1101_TXBYTES = 0x3A;
inline constexpr uint8_t CC1101_RXBYTES = 0x3B;

// CC1101 FIFOs
inline constexpr uint8_t CC1101_PATABLE = 0x3E;
inline constexpr uint8_t CC1101_TXFIFO = 0x3F;
inline constexpr uint8_t CC1101_RXFIFO = 0x3F;

// CC1101 MARC states
inline constexpr uint8_t MARCSTATE_IDLE = 0x01;
inline constexpr uint8_t MARCSTATE_RX = 0x0D;
inline constexpr uint8_t MARCSTATE_RXFIFO_OVERFLOW = 0x11;

}  // namespace wmbus_radio
}  // namespace esphome

