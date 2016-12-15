#include "U8glib.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);

void setup() {
  u8g.setFont(u8g_font_unifont);
  u8g.setColorIndex(255);
}

void draw(String text)
{
  u8g.firstPage();
  do {
    u8g.setPrintPos(0, 80);
    u8g.print(text);
  } while (u8g.nextPage());
}

void drawInt(int text)
{
  u8g.firstPage();
  do {
    u8g.setPrintPos(0, 20);
    u8g.print(text);
  } while (u8g.nextPage());
}

void loop() {
  String text = "puto";
  drawInt(8);
  delay(500);
  draw(text);
}
