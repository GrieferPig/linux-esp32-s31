/* SPDX-License-Identifier: GPL-2.0-only */
/* Private service role, distinct from a managed payload task/stop trampoline. */

struct s31_native_service_context {
    struct task_struct *thread;
    struct esp32s31_radio_domain *domain;
    struct s31_blob_context blob;
    bool admitted, active;

};
static struct s31_native_service_context s31_service_context;

static bool s31_service_data(const void *data)
{
    if (data != &s31_service_context) return false;
    if ((!s31_service_context.admitted) ||
        s31_service_context.thread != current ||
        s31_service_context.domain != READ_ONCE(s31_native_domain))
        panic("esp32s31-radio: admitted service identity corrupted");
    return true;
}

static bool s31_current_service(void)
{
    return (s31_service_data(esp32s31_radio_task_current_data(READ_ONCE(s31_native_domain))));
}
