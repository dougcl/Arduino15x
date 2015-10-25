// Copyright (c) 2010, Peter Barrett
/*
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#ifndef __USBDESC_H__
#define __USBDESC_H__

#define CDC_ENABLED
#ifndef __SAM3S4A__ //tackle HID later
#define HID_ENABLED
#endif

#ifdef CDC_ENABLED
#define CDC_INTERFACE_COUNT	2
#define CDC_ENPOINT_COUNT	3
#else
#define CDC_INTERFACE_COUNT	0
#define CDC_ENPOINT_COUNT	0
#endif

#ifdef HID_ENABLED
#define HID_INTERFACE_COUNT	1
#define HID_ENPOINT_COUNT	1
#else
#define HID_INTERFACE_COUNT	0
#define HID_ENPOINT_COUNT	0
#endif

#define CDC_ACM_INTERFACE	0	// CDC ACM
#define CDC_DATA_INTERFACE	1	// CDC Data
#ifdef __SAM3S4A__
//bump endpoints up to 3/4/5 for CDC because 1/2/3 won't work.
 #define CDC_FIRST_ENDPOINT	3 
#else
 #define CDC_FIRST_ENDPOINT	1
#endif
#define CDC_ENDPOINT_ACM	(CDC_FIRST_ENDPOINT)						// CDC First
#define CDC_ENDPOINT_OUT	(CDC_FIRST_ENDPOINT+1)
#define CDC_ENDPOINT_IN		(CDC_FIRST_ENDPOINT+2)


#define HID_INTERFACE		(CDC_ACM_INTERFACE + CDC_INTERFACE_COUNT)	// HID Interface
#define HID_FIRST_ENDPOINT	(CDC_FIRST_ENDPOINT + CDC_ENPOINT_COUNT)
#define HID_ENDPOINT_INT	(HID_FIRST_ENDPOINT)


#define INTERFACE_COUNT		(MSC_INTERFACE + MSC_INTERFACE_COUNT)

#ifdef CDC_ENABLED
#define CDC_RX CDC_ENDPOINT_OUT
#define CDC_TX CDC_ENDPOINT_IN
#endif

#ifdef HID_ENABLED
#define HID_TX HID_ENDPOINT_INT
#endif

#define IMANUFACTURER	1
#define IPRODUCT		2

#endif /* __USBDESC_H__ */
