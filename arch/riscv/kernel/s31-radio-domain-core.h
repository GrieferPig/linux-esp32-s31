/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef S31_RADIO_DOMAIN_CORE_H
#define S31_RADIO_DOMAIN_CORE_H
#include <linux/errno.h>
#include <linux/types.h>

#define S31_DOMAIN_TASK 0U
#define S31_DOMAIN_FOREIGN 1U
#define S31_DOMAIN_IRQ 2U
#define S31_DOMAIN_REFS 3U

/* The caller serializes every transition. Counts describe owned resources,
 * not radio execution exclusion: no domain lock spans a vendor callback. */
struct s31_domain_state {
    unsigned int refs[S31_DOMAIN_REFS];
    unsigned int closing;
    unsigned int task_admitted; /* errno binding closes permanently at first task admission. */
};

static inline int s31_domain_get(struct s31_domain_state *s, unsigned int kind)
{
    if (kind >= S31_DOMAIN_REFS) return -EINVAL;
    if (s->closing) return -ESHUTDOWN;
    if (s->refs[kind] == ~0U) return -EOVERFLOW;
    s->refs[kind]++;
    if (kind == S31_DOMAIN_TASK) s->task_admitted = 1;
    return 0;
}

static inline int s31_domain_put(struct s31_domain_state *s, unsigned int kind)
{
    if (kind >= S31_DOMAIN_REFS || !s->refs[kind]) return -EINVAL;
    s->refs[kind]--;
    return 0;
}

static inline int s31_domain_destroyable(const struct s31_domain_state *s)
{
    unsigned int i;
    if (!s->closing) return -EINVAL;
    for (i = 0; i < S31_DOMAIN_REFS; i++)
        if (s->refs[i]) return -EBUSY;
    return 0;
}
#endif
