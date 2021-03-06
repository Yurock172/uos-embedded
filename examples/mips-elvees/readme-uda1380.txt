UDA1380 - микросхема для записи и воспроизведения аудио. Спецификация на 
микросхему лежит в подпапке doc данной папки.

test-uda1380 демонстрирует воспроизведение аудио в формате WAVE (проверено на
44,1 КГц 16-бит стерео) на отладочной плате NVCOM-02T через эту микросхему. 
Для передачи аудио данных в UDA1380 используется интерфейс I2S, для управления -
I2C.

Работа теста проверялась с использованием отладочной платы UDA1380 Board:
http://www.aliexpress.com/item/UDA1380-Board-UDA-1380-Module-Stereo-Audio-Codecs-Decoder-Based-on-I2S-Interface/548266169.html
Принципиальная электрическая схема платы лежит в подпапке doc данной папки.

Подключение:
==============================================
UDA1380 Board       -       NVCOM-02T Ev.Board
==============================================
VCC                 -       XP8-1
GND                 -       XP8-39
SDA                 -       XP8-6
SCL                 -       XP8-5
TXCLK               -       XP6-12
TXWS                -       XP6-2
TXSDA               -       XP6-4


Разъём XP8 также называется "VIDEO PORT", а XP6 - "LINK PORT 2".

На отладочной плате UDA1380 Board джампера MCLK разомкнуты.
Звук прослушивался через наушники, подключённые к разъёму HEADPHONE.

Для сборки теста необходимо в target.cfg установить следующие значения
переменных:
CODE_PLACE	= FLASH
DATA_PLACE	= CRAM
LOADER		= STANDALONE

В Makefile указать основной тест test-uda1380:
TEST		= test-uda1380

Пересобрать проекты:
make clean; make

Далее необходимо подключить в JTAG-адаптер к отладочной плате NVCOM02T, 
запитать плату и запустить скрипт uda1380.sh:

./uda1380.sh

который включит в образ звуковой файл wave.wav и прошьёт его в плату.

