/* SPDX-License-Identifier: GPL-2.0 */
#ifndef ESP32S31_RADIO_HCI_FRAME_H
#define ESP32S31_RADIO_HCI_FRAME_H
/* TX H4 payload (type supplied separately). Closed VHCI asserts on unknown
 * types and reads protocol headers without validating the caller's extent.
 * Inspect only bounded bytes; no controller call, credit use or ring mutation.
 */
static inline bool s31_hci_frame_valid(u8 type, const u8 *data, size_t length)
{
 size_t header, payload;
 if (!data) return false;
 switch (type) {
 case 1: /* HCI command: opcode16, parameter length8. */
 case 3: /* SCO: handle16, data length8. */
  if (length < 3) return false;
  header = 3; payload = data[2];
  break;
 case 2: /* ACL: handle/flags16, data length16. */
 case 5: /* ISO: handle/flags16, Data_Total_Length. Max extent is 1029. */
  if (length < 4) return false;
  header = 4; payload = data[2] | ((size_t)data[3] << 8);
  break;
 default: /* Events are controller-to-host; other types are unsupported. */
  return false;
 }
 return payload == length - header;
}
#endif
