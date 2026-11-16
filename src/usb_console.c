#include "board.h"
#include "app.h"
#include "config.h"
#include "tusb.h"
#include <string.h>

/* 仅供本地原型开发使用的示例VID/PID，产品发布必须申请/分配合法标识。 */
static const tusb_desc_device_t device_descriptor={
    .bLength=sizeof(tusb_desc_device_t),.bDescriptorType=TUSB_DESC_DEVICE,.bcdUSB=0x0200,
    .bDeviceClass=TUSB_CLASS_MISC,.bDeviceSubClass=MISC_SUBCLASS_COMMON,.bDeviceProtocol=MISC_PROTOCOL_IAD,
    .bMaxPacketSize0=64,.idVendor=0xcafe,.idProduct=0x4011,.bcdDevice=0x0100,
    .iManufacturer=1,.iProduct=2,.iSerialNumber=3,.bNumConfigurations=1
};
static const uint8_t configuration_descriptor[]={
    TUD_CONFIG_DESCRIPTOR(1,2,0,TUD_CONFIG_DESC_LEN+TUD_CDC_DESC_LEN,TUSB_DESC_CONFIG_ATT_SELF_POWERED,0),
    TUD_CDC_DESCRIPTOR(0,4,0x81,8,0x02,0x82,64)
};
uint8_t const *tud_descriptor_device_cb(void) { return (const uint8_t *)&device_descriptor; }
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) { (void)index; return configuration_descriptor; }
uint16_t const *tud_descriptor_string_cb(uint8_t index,uint16_t langid) {
    (void)langid;
    static uint16_t descriptor[40];
    static const char *const strings[]={"","CM Motor Lab","CM ODrive FOC Console","CM-ODRIVE-0001","FOC CDC"};
    unsigned length;
    if (!index) { descriptor[1]=0x0409; length=1; }
    else {
        if (index>=sizeof(strings)/sizeof(strings[0])) return NULL;
        length=(unsigned)strlen(strings[index]);
        if (length>39) length=39;
        for (unsigned i=0;i<length;i++) descriptor[1+i]=(uint8_t)strings[index][i];
    }
    descriptor[0]=(uint16_t)((TUSB_DESC_STRING<<8)|(2*length+2));
    return descriptor;
}
/* 不依赖VBUS检测判断掉线，命令心跳超时独立停止运动。 */
static char line[96];
static unsigned length;
static bool invalid_line;

static void reset_line(void) {
    length=0;
    invalid_line=false;
}

static void serial_disconnected(void) {
    reset_line();
    if (g_app.motor[0].state!=IDLE||g_app.motor[1].state!=IDLE) app_fault(FAULT_TIMEOUT);
}

void tud_umount_cb(void) { serial_disconnected(); }

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    serial_disconnected();
}

void tud_cdc_line_state_cb(uint8_t interface, bool dtr, bool rts) {
    (void)interface;
    (void)rts;
    static bool was_open;
    if (was_open && !dtr) serial_disconnected();
    was_open=dtr;
}

void console_init(void) { board_usb_init(); }

void console_poll(void) {
    static char reply[768];
    static bool ever_mounted=false;
    tud_task();
    if (tud_mounted()) ever_mounted=true;
    if (USB_BOOT_DFU_TIMEOUT_MS&&!ever_mounted&&millis()>USB_BOOT_DFU_TIMEOUT_MS&&!g_app.prepared)
        board_request_dfu();
    /* Bound each pass so serial input does not monopolize the encoder polling task. */
    for (unsigned budget=0;budget<32 && tud_cdc_available();budget++) {
        unsigned char ch=(unsigned char)tud_cdc_read_char();
        if (ch=='\r'||ch=='\n') {
            if (length||invalid_line) {
                reply[0]=0;
                if (invalid_line) strcpy(reply,"ERR invalid ASCII line or line too long\r\n");
                else { line[length]=0; (void)app_command(line,reply,sizeof(reply)); }
                size_t size=strlen(reply);
                /* Keep replies whole and execute STOP even when the host stops reading. */
                if (size && tud_cdc_write_available()>=size) (void)tud_cdc_write(reply,size);
                reset_line();
            }
        } else if (ch=='\b'||ch==127) {
            if (length) length--;
        } else if ((ch>=32&&ch<127)||ch=='\t') {
            if (length<sizeof(line)-1) line[length++]=(char)ch; else invalid_line=true;
        } else {
            invalid_line=true;
        }
    }
    tud_cdc_write_flush();
}
