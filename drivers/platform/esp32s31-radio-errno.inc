/* SPDX-License-Identifier: GPL-2.0 */
/* Native domain tasks bank Picolibc's relocated cell at real Linux context
 * switches. Foreign windows retain their fenced enter/leave protocol. IRQ
 * nesting preserves the interrupted cell independently of either task kind. */
static int *s31_errno_current_bank(void);

static void s31_errno_enter(struct s31_blob_context *context)
{
 int *slot = READ_ONCE(s31_errno_slot);
 if (!slot) return; /* Runtime startup before registration. */
 if (!s31_linux_blob_held_by_current() || context->errno_active)
  panic("esp32s31-radio: invalid errno installation");
 if (esp32s31_radio_task_errno_native(READ_ONCE(s31_native_domain))) {
  context->errno_native = true;
  context->errno_active = true;
  return;
 }
 context->errno_bank = s31_errno_current_bank();
 WRITE_ONCE(*slot, READ_ONCE(*context->errno_bank));
 context->errno_active = true;
}

static void s31_errno_leave(struct s31_blob_context *context)
{
 if (!context->errno_active) return;
 if (!s31_linux_blob_held_by_current() || !s31_errno_slot)
  panic("esp32s31-radio: errno checkpoint lost owner");
 if (context->errno_native) {
  if (context->errno_bank ||
      !esp32s31_radio_task_errno_native(READ_ONCE(s31_native_domain)))
   panic("esp32s31-radio: native errno checkpoint lost owner");
  context->errno_native = false;
  context->errno_active = false;
  return;
 }
 if (!context->errno_bank)
  panic("esp32s31-radio: errno checkpoint lost bank");
 WRITE_ONCE(*context->errno_bank, READ_ONCE(*s31_errno_slot));
 context->errno_active = false;
 context->errno_bank = NULL;
}

static void s31_errno_register(int *slot, struct s31_blob_context *context)
{
 struct esp32s31_radio_domain *domain = READ_ONCE(s31_native_domain);
 if ((!domain))
  panic("esp32s31-radio: native errno registration has no domain");
 if (!s31_radio_fw_errno_slot_valid(slot) ||
     !context || !context->stack_switched || !s31_linux_blob_held_by_current() ||
     READ_ONCE(s31_errno_slot))
  panic("esp32s31-radio: invalid errno cell registration");
 if (domain && esp32s31_radio_domain_bind_errno(domain, slot))
  panic("esp32s31-radio: native errno binding rejected");
 WRITE_ONCE(s31_errno_slot, slot);
 s31_errno_enter(context);
}

static int s31_errno_irq_save(void)
{
 return s31_errno_slot ? READ_ONCE(*s31_errno_slot) : 0;
}

static void s31_errno_irq_restore(int value)
{
 if (s31_errno_slot) WRITE_ONCE(*s31_errno_slot, value);
}
