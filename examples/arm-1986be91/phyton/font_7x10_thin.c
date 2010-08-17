/*============================================================================================
 *
 *  (C) 2010, Phyton
 *
 *  Демонстрационный проект для 1986BE91 testboard
 *
 *  Данное ПО предоставляется "КАК ЕСТЬ", т.е. исключительно как пример, призванный облегчить
 *  пользователям разработку их приложений для процессоров Milandr 1986BE91T1. Компания Phyton
 *  не несет никакой ответственности за возможные последствия использования данного, или
 *  разработанного пользователем на его основе, ПО.
 *
 *--------------------------------------------------------------------------------------------
 *
 *  Файл Font_7x10_thin.c: Шрифт 7 на 10 пикселей тонкий.
 *  Аналог DOSApp-105 (7 на 12) Microsoft Windows (обрезаны две нижние строки).
 *
 *============================================================================================*/

#include "font_defs.h"

/* Изображение символа хранится в формате:                                  */
/* Каждый байт описывает все столбцы изображения верхних 8 строк символа.   */
/* Столбцы описываются слева-направо.                                       */
/* Младший бит байта описывается верхнюю строку стобца,                     */
/* старший бит - нижнюю строку.                                             */
/* Затем все повторяется для всех столбцов нижних 8 строк символа.          */

static const unsigned char Font_7x10_thin_Data[] = {
  /* 0x00 - пустое знакоместо.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x01 - лицо с улыбкой.*/
  0x78, 0xa4, 0x4a, 0x42, 0x4a, 0xa4, 0x78,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x02 - лицо с улыбкой закрашенное.*/
  0x78, 0xdc, 0xb6, 0xbe, 0xb6, 0xdc, 0x78,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x03 - червы (сердце).*/
  0x1c, 0x3e, 0x7e, 0xfc, 0x7e, 0x3e, 0x1c,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x04 - бубны.*/
  0x10, 0x38, 0x7c, 0xfe, 0x7c, 0x38, 0x10,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x05 - крести.*/
  0x30, 0x78, 0x77, 0xbf, 0x77, 0x78, 0x30,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x06 - вини.*/
  0x38, 0x7c, 0x7e, 0xbf, 0x7e, 0x7c, 0x38,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x07 - закрашенный круг по центру.*/
  0x00, 0x70, 0xf8, 0xf8, 0xf8, 0x70, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x08 - закрашенный круг по центру в инверсии.*/
  0xff, 0x8f, 0x07, 0x07, 0x07, 0x8f, 0xff,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0x09 - незакрашенный круг по центру.*/
  0x00, 0x70, 0x88, 0x88, 0x88, 0x70, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x0a - незакрашенный круг по центру в инверсии (кольцо).*/
  0xff, 0x8f, 0x77, 0x77, 0x77, 0x8f, 0xff,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0x0b - мужской символ (круг со стрелкой вверх).*/
  0xe0, 0x10, 0x10, 0x1a, 0xe6, 0x0e, 0x00,   0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,

  /* 0x0c - женский символ (круг с крестом внизу).*/
  0x00, 0x4e, 0x51, 0xf1, 0x51, 0x4e, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x0d - нота I.*/
  0x00, 0x80, 0x80, 0xfe, 0x04, 0x38, 0x00,   0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x0e - нота II.*/
  0x80, 0x80, 0xfe, 0x0a, 0xc5, 0xc5, 0x7f,   0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x0f - солнце (круг с расходящимися лучами).*/
  0x10, 0xba, 0x44, 0xc7, 0x44, 0xba, 0x10,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x10 - толстая стрелка вправо.*/
  0x00, 0x00, 0xfc, 0xf8, 0x70, 0x20, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x11 - толстая стрелка влево.*/
  0x00, 0x20, 0x70, 0xf8, 0xfc, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,

  /* 0x12 - тонкая стрелка вверх-вниз.*/
  0x00, 0x44, 0xc6, 0xff, 0xc6, 0x44, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x13 - два восклицательных знака.*/
  0x00, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x14 - символ "Пи".*/
  0x00, 0x00, 0x1f, 0x11, 0xff, 0x01, 0xff,   0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01,

  /* 0x15 - символ параграфа.*/
  0x00, 0x96, 0x29, 0x29, 0x29, 0xd2, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x16 - широкое подчеркивание.*/
  0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x17 - тонкая стрелка вверх-вниз с подчеркиванием.*/
  0x00, 0x24, 0x66, 0xff, 0x66, 0x24, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x18 - тонкая стрелка вверх.*/
  0x00, 0x04, 0x06, 0xff, 0x06, 0x04, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x19 - тонкая стрелка вниз.*/
  0x00, 0x40, 0xc0, 0xff, 0xc0, 0x40, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x1a - тонкая стрелка вправо.*/
  0x20, 0x20, 0x20, 0xf8, 0x70, 0x20, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x1b - тонкая стрелка влево.*/
  0x20, 0x70, 0xf8, 0x20, 0x20, 0x20, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x1c - символ отступа.*/
  0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x1d - тонкая стрелка влево-вправо.*/
  0x20, 0x70, 0xf8, 0x20, 0xf8, 0x70, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x1e - толстая стрелка вверх.*/
  0xc0, 0xf0, 0xfc, 0xff, 0xfc, 0xf0, 0xc0,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x1f - толстая стрелка вниз.*/
  0x06, 0x1e, 0x7e, 0xfe, 0x7e, 0x1e, 0x06,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x20 - пробел (пустое знакоместо).*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x21 - восклицательный знак.*/
  0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x22 - двойная кавычка.*/
  0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x23 - решетка.*/
  0x00, 0x48, 0xfe, 0x48, 0xfe, 0x48, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x24 - доллар.*/
  0x00, 0x4c, 0x92, 0x93, 0x92, 0x64, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x25 - процент.*/
  0x00, 0x84, 0x4a, 0x24, 0x90, 0x48, 0x86,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x26 - амперсанд.*/
  0x00, 0xf6, 0x09, 0x09, 0x09, 0xf6, 0x30,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01,

  /* 0x27 - апостроф.*/
  0x00, 0x00, 0x00, 0x02, 0x06, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x28 - открывающая скобка.*/
  0x00, 0x7c, 0x82, 0x01, 0x01, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

  /* 0x29 - закрывающая скобка.*/
  0x00, 0x01, 0x01, 0x82, 0x7c, 0x00, 0x00,   0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x2a - звездочка (умножение).*/
  0x00, 0x54, 0x7c, 0x10, 0x7c, 0x54, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x2b - плюс.*/
  0x00, 0x10, 0x10, 0x7c, 0x10, 0x10, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x2c - запятая.*/
  0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,

  /* 0x2d - тире.*/
  0x00, 0x00, 0x20, 0x20, 0x20, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x2e - точка.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x2f - слеш слева-направо ('/').*/
  0x00, 0x80, 0x60, 0x18, 0x06, 0x01, 0x00,   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x30 - '0'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x31 - '1'.*/
  0x00, 0x04, 0x02, 0xff, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x32 - '2'.*/
  0x00, 0x86, 0x41, 0x21, 0x11, 0x0e, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x33 - '3'.*/
  0x00, 0x82, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x34 - '4'.*/
  0x00, 0x3e, 0x40, 0x40, 0x40, 0xfe, 0x40,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x35 - '5'.*/
  0x00, 0x8f, 0x09, 0x09, 0x09, 0xf1, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x36 - '6'.*/
  0x00, 0xfe, 0x11, 0x11, 0x11, 0xe2, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x37 - '7'.*/
  0x00, 0x01, 0x81, 0x61, 0x19, 0x07, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x38 - '8'.*/
  0x00, 0xee, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x39 - '9'.*/
  0x00, 0x0e, 0x11, 0x11, 0x91, 0x7e, 0x00,   0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,

  /* 0x3a - двоеточие.*/
  0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x3b - точка с запятой.*/
  0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x3c - меньше.*/
  0x00, 0x10, 0x28, 0x44, 0x82, 0x01, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x3d - равно.*/
  0x00, 0x48, 0x48, 0x48, 0x48, 0x48, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x3e - больше.*/
  0x00, 0x01, 0x82, 0x44, 0x28, 0x10, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x3f - вопросительный знак.*/
  0x00, 0x06, 0xa1, 0x11, 0x11, 0x0e, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x40 - "собака" ('@').*/
  0x00, 0x7c, 0x82, 0x01, 0x39, 0x45, 0x3e,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,

  /* 0x41 - 'A'.*/
  0x00, 0xfc, 0x22, 0x21, 0x22, 0xfc, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x42 - 'B'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x43 - 'C'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xc6, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x44 - 'D'.*/
  0x00, 0xff, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x45 - 'E'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0x01, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x46 - 'F'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0x01, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x47 - 'G'.*/
  0x00, 0xfe, 0x01, 0x01, 0x21, 0xe2, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x48 - 'H'.*/
  0x00, 0xff, 0x10, 0x10, 0x10, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x49 - 'I'.*/
  0x00, 0x00, 0x01, 0xff, 0x01, 0x00, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x4a - 'J'.*/
  0x00, 0xc0, 0x00, 0x00, 0x00, 0xff, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x4b - 'K'.*/
  0x00, 0xff, 0x10, 0x10, 0x28, 0xc7, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x4c - 'L'.*/
  0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x4d - 'M'.*/
  0x00, 0xff, 0x0c, 0x70, 0x70, 0x0c, 0xff,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01,

  /* 0x4e - 'N'.*/
  0x00, 0xff, 0x08, 0x10, 0x20, 0x40, 0xff,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01,

  /* 0x4f - 'O'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x50 - 'P'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0x0e, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x51 - 'Q'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03,

  /* 0x52 - 'R'.*/
  0x00, 0xff, 0x11, 0x11, 0x31, 0xce, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x53 - 'S'.*/
  0x00, 0xce, 0x11, 0x11, 0x11, 0xe6, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x54 - 'T'.*/
  0x00, 0x01, 0x01, 0xff, 0x01, 0x01, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x55 - 'U'.*/
  0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x56 - 'V'.*/
  0x00, 0x7f, 0x80, 0x00, 0x80, 0x7f, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x57 - 'W'.*/
  0x00, 0x7f, 0x80, 0x7c, 0x80, 0x7f, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x58 - 'X'.*/
  0x00, 0xc7, 0x28, 0x10, 0x28, 0xc7, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x59 - 'Y'.*/
  0x00, 0x0f, 0x10, 0xe0, 0x10, 0x0f, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x5a - 'Z'.*/
  0x00, 0x81, 0x41, 0x39, 0x05, 0x03, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x5b - '['.*/
  0x00, 0x00, 0xff, 0x01, 0x01, 0x00, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x5c - '\'.*/
  0x00, 0x03, 0x0c, 0x30, 0xc0, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,

  /* 0x5d - ']'.*/
  0x00, 0x00, 0x01, 0x01, 0xff, 0x00, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x5e - '^'.*/
  0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x5f - '_'.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00,

  /* 0x60 - обратный апостроф.*/
  0x00, 0x00, 0x07, 0x03, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x61 - 'a'.*/
  0x00, 0xc0, 0x28, 0x28, 0x28, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x62 - 'b'.*/
  0x00, 0xff, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x63 - 'c'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0x90, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x64 - 'd'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xff, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x65 - 'e'.*/
  0x00, 0xf0, 0x48, 0x48, 0x48, 0x70, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x66 - 'f'.*/
  0x00, 0x08, 0xfe, 0x09, 0x09, 0x01, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x67 - 'g'.*/
  0x00, 0x30, 0x48, 0x48, 0x48, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x68 - 'h'.*/
  0x00, 0xff, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x69 - 'i'.*/
  0x00, 0x00, 0x00, 0x09, 0xf9, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0x6a - 'j'.*/
  0x00, 0x80, 0x00, 0x0a, 0xfa, 0x00, 0x00,   0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,

  /* 0x6b - 'k'.*/
  0x00, 0xff, 0x20, 0x20, 0xd0, 0x08, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x6c - 'l'.*/
  0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0x6d - 'm'.*/
  0x00, 0xf8, 0x10, 0xe0, 0x10, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x6e - 'n'.*/
  0x00, 0xf8, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x6f - 'o'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x70 - 'p'.*/
  0x00, 0xf8, 0x48, 0x48, 0x48, 0x30, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x71 - 'q'.*/
  0x00, 0x30, 0x48, 0x48, 0x48, 0xf8, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x72 - 'r'.*/
  0x00, 0xf8, 0x10, 0x08, 0x08, 0x10, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x73 - 's'.*/
  0x00, 0x10, 0x28, 0x28, 0x28, 0xc8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x74 - 't'.*/
  0x00, 0x08, 0xfe, 0x08, 0x08, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0x75 - 'u'.*/
  0x00, 0xf8, 0x00, 0x00, 0x00, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x76 - 'v'.*/
  0x00, 0x78, 0x80, 0x00, 0x80, 0x78, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x77 - 'w'.*/
  0x00, 0xf8, 0x80, 0xf0, 0x80, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x78 - 'x'.*/
  0x00, 0x08, 0x90, 0x60, 0x90, 0x08, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x79 - 'y'.*/
  0x00, 0x38, 0x40, 0x40, 0x40, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x7a - 'z'.*/
  0x00, 0x88, 0x48, 0x48, 0x28, 0x18, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x7b - '{'.*/
  0x00, 0x10, 0xee, 0x01, 0x01, 0x01, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0x7c - '|'.*/
  0x00, 0x00, 0x00, 0xcf, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x7d - '}'.*/
  0x00, 0x01, 0x01, 0xee, 0x10, 0x00, 0x00,   0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x7e - '~'.*/
  0x00, 0x06, 0x02, 0x07, 0x02, 0x03, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x7f - "домик".*/
  0x00, 0xf0, 0x88, 0x84, 0x88, 0xf0, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x80 - сетка из точек разреженная.*/
  0x44, 0x00, 0x11, 0x44, 0x00, 0x11, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00,

  /* 0x81 - сетка из точек густая.*/
  0xaa, 0x00, 0x55, 0xaa, 0x00, 0x55, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00,

  /* 0x82 - сетка из черточек.*/
  0xaa, 0x55, 0x55, 0xaa, 0x55, 0xaa, 0x55,   0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01,

  /* 0x83 - псевдографика - вертикальная черта.*/
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x84 - псевдографика - вертикальная черта с отводом по центру влево.*/
  0x20, 0x20, 0x20, 0xff, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x85 - псевдографика - вертикальная черта с двойным отводом по центру влево.*/
  0x50, 0x50, 0x50, 0xff, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x86 - псевдографика - двойная вертикальная черта с отводом по центру влево.*/
  0x20, 0x20, 0xff, 0x00, 0xff, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x87 - псевдографика - верхний правый угол с двойной вертикальной чертой.*/
  0x20, 0x20, 0xe0, 0x20, 0xe0, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x88 - псевдографика - верхний правый угол с двойной горизонтальной чертой.*/
  0x50, 0x50, 0x50, 0xf0, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x89 - псевдографика - двойная вертикальная черта с двойным отводом по центру влево.*/
  0x50, 0x50, 0xdf, 0x00, 0xff, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x8a - псевдографика - двойная вертикальная черта.*/
  0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x8b - псевдографика - двойной верхний правый угол.*/
  0x50, 0x50, 0xd0, 0x10, 0xf0, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x8c - псевдографика - двойной нижний правый угол.*/
  0x50, 0x50, 0x5f, 0x40, 0x7f, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x8d - псевдографика - нижний правый угол с двойной вертикальной чертой.*/
  0x20, 0x20, 0x3f, 0x20, 0x3f, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x8e - псевдографика - нижний правый угол с двойной горизонтальной чертой.*/
  0x50, 0x50, 0x50, 0x7f, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x8f - псевдографика - верхний правый угол.*/
  0x20, 0x20, 0x20, 0xe0, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x90 - псевдографика - нижний левый угол.*/
  0x00, 0x00, 0x00, 0x3f, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x91 - псевдографика - горизонтальная черта с отводом по центру вверх.*/
  0x20, 0x20, 0x20, 0x3f, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x92 - псевдографика - горизонтальная черта с отводом по центру вниз.*/
  0x20, 0x20, 0x20, 0xe0, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x93 - псевдографика - вертикальная черта с отводом по центру вправо.*/
  0x00, 0x00, 0x00, 0xff, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x94 - псевдографика - горизонтальная черта по центру.*/
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x95 - псевдографика - перекрестие.*/
  0x20, 0x20, 0x20, 0xff, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x96 - псевдографика - вертикальная черта с двойным отводом по центру вправо.*/
  0x00, 0x00, 0x00, 0xff, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x97 - псевдографика - двойная вертикальная черта с отводом по центру вправо.*/
  0x00, 0x00, 0xff, 0x00, 0xff, 0x20, 0x20,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x98 - псевдографика - двойной нижний левый угол.*/
  0x00, 0x00, 0x7f, 0x40, 0x5f, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x99 - псевдографика - двойной верхний левый угол.*/
  0x00, 0x00, 0xf0, 0x10, 0xd0, 0x50, 0x50,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x9a - псевдографика - двойная горизонтальная черта с двойным отводом по центру вверх.*/
  0x50, 0x50, 0x5f, 0x40, 0x5f, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x9b - псевдографика - двойная горизонтальная черта с двойным отводом по центру вниз.*/
  0x50, 0x50, 0xd0, 0x10, 0xd0, 0x50, 0x50,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x9c - псевдографика - двойная вертикальная черта с двойным отводом по центру вправо.*/
  0x00, 0x00, 0xff, 0x00, 0xdf, 0x50, 0x50,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x9d - псевдографика - двойная горизонтальная черта по центру.*/
  0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x9e - псевдографика - двойное перекрестие.*/
  0x50, 0x50, 0xdf, 0x00, 0xdf, 0x50, 0x50,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x9f - псевдографика - двойная горизонтальная черта с отводом по центру вверх.*/
  0x50, 0x50, 0x50, 0x5f, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xa0 - псевдографика - горизонтальная черта с двойным отводом по центру вверх.*/
  0x20, 0x20, 0x3f, 0x20, 0x3f, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xa1 - псевдографика - двойная горизонтальная черта с отводом по центру вниз.*/
  0x50, 0x50, 0x50, 0xd0, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xa2 - псевдографика - горизонтальная черта с двойным отводом по центру вниз.*/
  0x20, 0x20, 0xe0, 0x20, 0xe0, 0x20, 0x20,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0xa3 - псевдографика - нижний левый угол, двойная вертикальная черта.*/
  0x00, 0x00, 0x3f, 0x20, 0x3f, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xa4 - псевдографика - нижний левый угол, двойная горизонтальная черта.*/
  0x00, 0x00, 0x00, 0x7f, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xa5 - псевдографика - верхний левый угол, двойная горизонтальная черта.*/
  0x00, 0x00, 0x00, 0xf0, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xa6 - псевдографика - верхний левый угол, двойная вертикальная черта.*/
  0x00, 0x00, 0xe0, 0x20, 0xe0, 0x20, 0x20,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0xa7 - псевдографика - двойная вертикальная черта по центру с отводами вправо и влево.*/
  0x20, 0x20, 0xff, 0x00, 0xff, 0x20, 0x20,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0xa8 - 'Ё'.*/
  0x00, 0xfd, 0x25, 0x24, 0x25, 0x05, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xa9 - псевдографика - двойная горизонтальная черта по центру с отводами вверх и вниз.*/
  0x50, 0x50, 0x50, 0xdf, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xaa - псевдографика - нижний правый угол.*/
  0x20, 0x20, 0x20, 0x3f, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xab - псевдографика - верхний левый угол.*/
  0x00, 0x00, 0x00, 0xe0, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xac - псевдографика - закрашенное знакоместо.*/
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xad - псевдографика - закрашенная нижняя половина знакоместа.*/
  0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xae - псевдографика - закрашенная левая половина знакоместа.*/
  0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,   0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,

  /* 0xaf - псевдографика - закрашенная правая половина знакоместа.*/
  0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,

  /* 0xb0 - псевдографика - закрашенная верхняя половина знакоместа.*/
  0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xb1 - зеркальная 'Э'.*/
  0x00, 0xfe, 0x11, 0x11, 0x11, 0x82, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb2 - зеркальная 'э'.*/
  0x00, 0xf0, 0x28, 0x28, 0x08, 0x90, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb3 - 'I' с двумя точками вверху.*/
  0x00, 0x01, 0x01, 0xfc, 0x01, 0x01, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb4 - 'i' с двумя точками вверху.*/
  0x00, 0x02, 0x0a, 0x08, 0xfa, 0x02, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xb5 - 'Y' с тильдой вверху ('~').*/
/*  0x00, 0x9c, 0x21, 0x22, 0x21, 0xfc, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,*/

/* 0xb5 - символ, отображенный на кнопке включения звука "(*)".*/
0x00, 0x78, 0x84, 0x30, 0x30, 0x84, 0x78,    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xb6 - 'y' с тильдой вверху ('~').*/
  0x00, 0x38, 0x42, 0x44, 0x42, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb7 - маленький кружок вверху.*/
  0x00, 0x0e, 0x11, 0x11, 0x0e, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xb8 - 'ё'.*/
  0x00, 0xf0, 0x2b, 0x28, 0x2b, 0xb0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb9 - большой закрашенный круг по центру.*/
  0x00, 0x00, 0x08, 0x1c, 0x08, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xba - маленький закрашенный круг по центру.*/
  0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xbb - символ корня квадратного.*/
  0x00, 0x40, 0x80, 0x00, 0xfe, 0x02, 0x02,   0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

  /* 0xbc - '№'.*/
  0xff, 0x04, 0x38, 0x40, 0xff, 0x19, 0x00,   0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,

  /* 0xbd - "солнце".*/
  0x74, 0x88, 0x04, 0x04, 0x88, 0x74, 0x00,   0x01, 0x00, 0x01, 0x01, 0x00, 0x01, 0x00,

  /* 0xbe - закрашенный квадрат по центру.*/
  0x00, 0x00, 0x38, 0x38, 0x38, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xbf - пустое знакоместо.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xc0 - 'А'.*/
  0x00, 0xfc, 0x22, 0x21, 0x22, 0xfc, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xc1 - 'Б'.*/
  0x00, 0xff, 0x09, 0x09, 0x09, 0xf1, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xc2 - 'В'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xc3 - 'Г'.*/
  0x00, 0xff, 0x01, 0x01, 0x01, 0x01, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xc4 - 'Д'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xff, 0x00,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xc5 - 'Е'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0x01, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xc6 - 'Ж'.*/
  0x00, 0xef, 0x10, 0xff, 0x10, 0xef, 0x00,   0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,

  /* 0xc7 - 'З'.*/
  0x00, 0x82, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xc8 - 'И'.*/
  0x00, 0xff, 0x40, 0x30, 0x08, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xc9 - 'Й'.*/
  0x00, 0xff, 0x40, 0x33, 0x08, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xca - 'К'.*/
  0x00, 0xff, 0x10, 0x10, 0x28, 0xc7, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xcb - 'Л'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xcc - 'М'.*/
  0x00, 0xff, 0x0c, 0x70, 0x0c, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xcd - 'Н'.*/
  0x00, 0xff, 0x10, 0x10, 0x10, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xce - 'О'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xcf - 'П'.*/
  0x00, 0xff, 0x01, 0x01, 0x01, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xd0 - 'Р'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0x0e, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xd1 - 'С'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xc6, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xd2 - 'Т'.*/
  0x00, 0x01, 0x01, 0xff, 0x01, 0x01, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xd3 - 'У'.*/
  0x00, 0x8f, 0x10, 0x10, 0x10, 0xff, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xd4 - 'Ф'.*/
  0x00, 0x3c, 0x42, 0xff, 0x42, 0x3c, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xd5 - 'Х'.*/
  0x00, 0xc7, 0x28, 0x10, 0x28, 0xc7, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xd6 - 'Ц'.*/
  0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xd7 - 'Ч'.*/
  0x00, 0x0f, 0x10, 0x10, 0x10, 0xff, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xd8 - 'Ш'.*/
  0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xd9 - 'Щ'.*/
  0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xda - 'Ъ'.*/
  0x01, 0xff, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xdb - 'Ы'.*/
  0x00, 0xff, 0x10, 0xe0, 0x00, 0xff, 0x00,   0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00,

  /* 0xdc - 'Ь'.*/
  0x00, 0xff, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xdd - 'Э'.*/
  0x00, 0x82, 0x11, 0x11, 0x11, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xde - 'Ю'.*/
  0x00, 0xff, 0x18, 0xff, 0x01, 0xff, 0x00,   0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0xdf - 'Я'.*/
  0x00, 0xee, 0x11, 0x11, 0x11, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xe0 - 'а'.*/
  0x00, 0xc0, 0x28, 0x28, 0x28, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xe1 - 'б'.*/
  0x00, 0xf8, 0x24, 0x24, 0x24, 0xc2, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xe2 - 'в'.*/
  0x00, 0xf8, 0x28, 0x28, 0x28, 0xd0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xe3 - 'г'.*/
  0x00, 0xf8, 0x08, 0x08, 0x08, 0x08, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xe4 - 'д'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xf8, 0x00,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xe5 - 'е'.*/
  0x00, 0xf0, 0x28, 0x28, 0x28, 0xb0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xe6 - 'ж'.*/
  0x00, 0xd8, 0x20, 0xf8, 0x20, 0xd8, 0x00,   0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,

  /* 0xe7 - 'з'.*/
  0x00, 0x90, 0x08, 0x28, 0x28, 0xd0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xe8 - 'и'.*/
  0x00, 0xf8, 0x80, 0x40, 0x20, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xe9 - 'й'.*/
  0x00, 0xf8, 0x80, 0x44, 0x22, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xea - 'к'.*/
  0x00, 0xf8, 0x20, 0x20, 0x50, 0x88, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xeb - 'л'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xec - 'м'.*/
  0x00, 0xf8, 0x10, 0x60, 0x10, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xed - 'н'.*/
  0x00, 0xf8, 0x20, 0x20, 0x20, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xee - 'о'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xef - 'п'.*/
  0x00, 0xf8, 0x08, 0x08, 0x08, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xf0 - 'р'.*/
  0x00, 0xf8, 0x48, 0x48, 0x48, 0x30, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xf1 - 'с'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0x90, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xf2 - 'т'.*/
  0x00, 0x08, 0x08, 0xf8, 0x08, 0x08, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xf3 - 'у'.*/
  0x00, 0x38, 0x40, 0x40, 0x40, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xf4 - 'ф'.*/
  0x00, 0x30, 0x48, 0xf8, 0x48, 0x30, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xf5 - 'х'.*/
  0x00, 0x08, 0x90, 0x60, 0x90, 0x08, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xf6 - 'ц'.*/
  0x00, 0xf8, 0x00, 0x00, 0x00, 0xf8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xf7 - 'ч'.*/
  0x00, 0x38, 0x40, 0x40, 0x40, 0xf8, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xf8 - 'ш'.*/
  0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xf9 - 'щ'.*/
  0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xfa - 'ъ'.*/
  0x08, 0xf8, 0x20, 0x20, 0x20, 0xc0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xfb - 'ы'.*/
  0x00, 0xf8, 0x20, 0xe0, 0x00, 0xf8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00,

  /* 0xfc - 'ь'.*/
  0x00, 0xf8, 0x20, 0x20, 0x20, 0xc0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xfd - 'э'.*/
  0x00, 0x90, 0x08, 0x28, 0x28, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xfe - 'ю'.*/
  0x00, 0xf8, 0x20, 0xf8, 0x08, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0xff - 'я'.*/
  0x00, 0xb0, 0x48, 0x48, 0x48, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00
};

FONT Font_7x10_thin = {
  10,                       /* Высота символа в пикселах.*/
  7,                        /* Ширина символа в пикселах.*/
  255,                      /* Число символов в шрифте.*/
  Font_7x10_thin_Data      /* Адрес таблицы описания символов шрифта.*/
};

/*============================================================================================
 * Конец файла Font_7x10_thin.c
 *============================================================================================*/
