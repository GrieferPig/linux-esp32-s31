/* SPDX-License-Identifier: GPL-2.0-only */
/* Cold admission follows exclusive metadata bootstrap. The private control
 * identity and exit-work lifetime remain until owner-side retirement. */
struct s31_blob_context *s31_linux_radio_service_context(void)
{
    return s31_current_service() ? &s31_service_context.blob : NULL;
}

int s31_linux_radio_service_admit_current(void)
{
    struct s31_foreign_owner *owner;
    struct esp32s31_radio_domain *domain;
    int initial_errno, ret;

    if ((current != s31_service_context.thread))
        return 0;
    if (s31_service_context.admitted) {
        if (!s31_current_service())
            panic("esp32s31-radio: service admission lost architecture identity");

        return 0;
    }
    if (!READ_ONCE(s31_errno_slot)) return 0; /* Exclusive metadata bootstrap is not done. */
    if (!s31_task_create_kernel_context() || s31_blob_active() ||
        s31_linux_blob_held_by_current() || current->migration_disabled)
        return -EWOULDBLOCK;
    domain = READ_ONCE(s31_native_domain);
    if (!domain) return -EINVAL;
    owner = s31_foreign_owner_get(current);
    if (!owner) return -EINVAL;
    if (READ_ONCE(owner->retiring)) {
        s31_foreign_owner_put(owner);
        return -ESHUTDOWN;
    }
    initial_errno = READ_ONCE(owner->errno_value);
    s31_foreign_owner_put(owner);
    s31_service_context.domain = domain;
    /* Initialize the driver role before kernel publication. A preempting
     * IRQ after publication can query it before this API returns. No role
     * is observable through the kernel until attach actually succeeds. */
    s31_service_context.admitted = true;
    ret = esp32s31_radio_domain_current_attach(domain, &s31_service_context,
                                              initial_errno);
    if (ret) {
        s31_service_context.admitted = false;
        s31_service_context.domain = NULL;
        return ret;
    }

    return 0;
}

int s31_linux_radio_service_retire_current(void)
{
    struct s31_foreign_owner *owner;
    struct esp32s31_radio_domain *domain;
    bool foreign_finished;
    int ret;

    if ((current != s31_service_context.thread)) {
        s31_service_priority_retire_current();
        return 0;
    }
    if (!s31_service_context.admitted) {
        s31_service_context.thread = NULL;
        s31_service_priority_retire_current();
        return 0;
    }
    if (!s31_current_service() || !s31_task_create_kernel_context() ||
        s31_blob_active() || s31_linux_blob_held_by_current() ||
        current->migration_disabled || s31_service_context.blob.preempt_fenced ||
        s31_service_context.blob.errno_active ||
        s31_service_context.blob.stack_switched)
        return -EBUSY;
    owner = s31_foreign_owner_get(current);
    if (!owner) return -EINVAL;
    foreign_finished = completion_done(&owner->exit.done) &&
                       READ_ONCE(owner->retiring) && owner->control_only;
    s31_foreign_owner_put(owner);
    if (!foreign_finished) return -EBUSY;
    s31_native_mutex_assert_current_quiescent();
    domain = s31_service_context.domain;
    ret = esp32s31_blob_fp_detach(current);
    if (ret) return ret;
    ret = esp32s31_radio_domain_task_retire(domain);
    if (ret) panic("esp32s31-radio: detached service retirement failed (%d)", ret);
    s31_service_context.admitted = false;
    s31_service_context.thread = NULL;
    s31_service_context.domain = NULL;
    s31_service_priority_retire_current();

    return 0;
}
