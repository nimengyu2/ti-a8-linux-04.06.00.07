/* zutil.h -- internal interface and configuration of the compression library
 * Copyright (C) 1995-1998 Jean-loup Gailly.
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/* WARNING: this file should *not* be used by applications. It is
   part of the implementation of the compression library and is
   subject to change. Applications should only use zlib.h.
 */

/* @(#) $Id: zutil.h,v 1.1 2000/01/01 03:32:23 davem Exp $ */

#ifndef _LIERDA_DEBUG_H
#define _LIERDA_DEBUG_H

#define LSD_ERR "ERROR"
#define LSD_WRN "WARNING"
#define LSD_DBG "DEBUG"
#define LSD_OK "OK"

#define M_LSD_DBG  1
#define M_LSD_AUDIO_DBG  1
#define M_LSD_AUDIO_MCBSP  1
#define M_LSD_FB_DBG  1


// all debug
#if(M_LSD_DBG >= 1)
#define lsd_dbg(level,format, arg...)	\
	printk("AAAA:%s:lsd all:file=%s,func=%s,line=%d:BBBB: " format ,\
		level,__FILE__,__FUNCTION__,__LINE__,## arg)
#else
#define lsd_dbg(level,format, arg...)		
#endif

// audio debug
#if(M_LSD_AUDIO_DBG >= 1)
#define lsd_audio_dbg(level,format, arg...) \
	printk("AAAA:%s:audio:file=%s,func=%s,line=%d:BBBB: " format ,\
		level,__FILE__,__FUNCTION__,__LINE__,## arg)
#else
#define lsd_audio_dbg(level,format, arg...) 
#endif

// fb debug
#if(M_LSD_FB_DBG >= 1)
#define lsd_fb_dbg(level,format, arg...) \
	printk("AAAA:%s:fb:file=%s,func=%s,line=%d:BBBB: " format ,\
		level,__FILE__,__FUNCTION__,__LINE__,## arg)
#else
#define lsd_fb_dbg(level,format, arg...) 
#endif

// eth debug
#if(M_LSD_ETH_DBG >= 1)
#define lsd_eth_dbg(level,format, arg...) \
	printk("AAAA:%s:fb:file=%s,func=%s,line=%d:BBBB: " format ,\
		level,__FILE__,__FUNCTION__,__LINE__,## arg)
#else
#define lsd_eth_dbg(level,format, arg...) 
#endif

// mmc debug
#if(M_LSD_MMC_DBG >= 1)
#define lsd_mmc_dbg(level,format, arg...) \
	printk("AAAA:%s:fb:file=%s,func=%s,line=%d:BBBB: " format ,\
		level,__FILE__,__FUNCTION__,__LINE__,## arg)
#else
#define lsd_mmc_dbg(level,format, arg...) 
#endif

// usb debug
#if(M_LSD_USB_DBG >= 1)
#define lsd_usb_dbg(level,format, arg...) \
	printk("AAAA:%s:fb:file=%s,func=%s,line=%d:BBBB: " format ,\
		level,__FILE__,__FUNCTION__,__LINE__,## arg)
#else
#define lsd_usb_dbg(level,format, arg...) 
#endif


#endif /* _Z_UTIL_H */
