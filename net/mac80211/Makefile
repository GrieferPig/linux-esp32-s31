# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_MAC80211) += mac80211.o

# mac80211 objects
mac80211-y := \
	main.o status.o \
	driver-ops.o \
	sta_info.o \
	wep.o \
	aead_api.o \
	wpa.o \
	scan.o offchannel.o \
	ht.o agg-tx.o agg-rx.o \
	vht.o \
	he.o \
	s1g.o \
	ibss.o \
	iface.o \
	link.o \
	rate.o \
	michael.o \
	tkip.o \
	aes_cmac.o \
	aes_gmac.o \
	fils_aead.o \
	cfg.o \
	ethtool.o \
	rx.o \
	spectmgmt.o \
	tx.o \
	key.o \
	util.o parse.o \
	wme.o \
	chan.o \
	trace.o mlme.o \
	tdls.o \
	ocb.o \
	airtime.o \
	eht.o

mac80211-$(CONFIG_MAC80211_LEDS) += led.o
mac80211-$(CONFIG_MAC80211_DEBUGFS) += \
	debugfs.o \
	debugfs_sta.o \
	debugfs_netdev.o \
	debugfs_key.o

mac80211-$(CONFIG_MAC80211_MESH) += \
	mesh.o \
	mesh_pathtbl.o \
	mesh_plink.o \
	mesh_hwmp.o \
	mesh_sync.o \
	mesh_ps.o

mac80211-$(CONFIG_PM) += pm.o

CFLAGS_trace.o := -I$(src)

rc80211_minstrel-y := \
	rc80211_minstrel_ht.o

rc80211_minstrel-$(CONFIG_MAC80211_DEBUGFS) += \
	rc80211_minstrel_ht_debugfs.o

mac80211-$(CONFIG_MAC80211_RC_MINSTREL) += $(rc80211_minstrel-y)

obj-y += tests/

mac80211-y += wbrf.o

ccflags-y += -DDEBUG

# S31 executes networking from XIP flash. Keep the receive hot path optimized
# for throughput while retaining the kernel's size optimization elsewhere.
ifeq ($(CONFIG_SOC_ESP32S31),y)
CFLAGS_rx.o += -O2
CFLAGS_aead_api.o += -O2
CFLAGS_wpa.o += -O2
CFLAGS_tx.o += -O2
CFLAGS_status.o += -O2
CFLAGS_util.o += -O2
CFLAGS_sta_info.o += -O2
endif
