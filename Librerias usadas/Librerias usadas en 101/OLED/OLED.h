#ifndef OLED_H
#define OLED_H





#define OLED_I2C_ADDRESS   0x3C

#define OLED_CONTROL_BYTE_CMD_SINGLE  0x80
#define OLED_CONTROL_BYTE_CMD_STREAM  0x00
#define OLED_CONTROL_BYTE_DATA_STREAM 0x40

#define OLED_CMD_SET_CONTRAST     0x81  // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM      0xA4
#define OLED_CMD_DISPLAY_ALLON      0xA5
#define OLED_CMD_DISPLAY_NORMAL     0xA6
#define OLED_CMD_DISPLAY_INVERTED     0xA7
#define OLED_CMD_DISPLAY_OFF      0xAE
#define OLED_CMD_DISPLAY_ON       0xAF

#define OLED_CMD_SET_MEMORY_ADDR_MODE 0x20  // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE   0x21  // can be used only in HORZ/VERT mode - follow with 0x00 + 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE     0x22

#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP    0xA1
#define OLED_CMD_SET_MUX_RATIO      0xA8  // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE    0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET   0xD3  // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP    0xDA  // follow with 0x12
#define OLED_CMD_NOP          0xE3  // NOP

#define OLED_CMD_SET_DISPLAY_CLK_DIV  0xD5  // follow with 0x80
#define OLED_CMD_SET_PRECHARGE      0xD9  // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT    0xDB  // follow with 0x30

#define OLED_CMD_SET_CHARGE_PUMP    0x8D  // follow with 0x14


class OLED
{
    private:
    //    static unsigned char kit [1024];

    public:

        OLED();
        void Init();


        void Yazaki();
        void JohnDeere();
        void DefinityFirst();


        void Mexico();
        void RoBorregos();

        void Kit();
        void KitNegro();
        void Meta();
        void Adelante();
        void Atras();
        void Derecha();
        void Izquierda();
        void Rampa();




};

#endif // OLED_H
