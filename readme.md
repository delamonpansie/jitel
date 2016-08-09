# Житель: Jive to Futaba telemetry

### Я не отвечаю за любые последствия. Все на свой страх и риск.

  Некоторое время назад я стал относительно счастливым обладателем
аппаратуры с телеметрией, производства всемирноизвестной в узких
кругах фирмы футаба.  К сожалению, эргономика данной телеметрии
оставляет желать лучшего. Мне не хватило удобной индикации о том, что
батарейка закончилась.  Штатное решение с ext.voltage датчиком не
выдерживает никакой критики: при любом нагруженном маневре напряжение
просаживается ниже порога и приводит к срабатыванию алярма.  Частично
решить проблему мог бы гистерезис, но он не предусмотрен футабой.
Более-менее эту проблему решает сторонний датчик eFuelGauge, который
считает mAh, но это плохо работает если используются батарейки разной
емкости.  Являсь давним пользователем jlog2, я знал, что из jive едет
отладочная информация, в которой есть как напряжение, так и
ток. Поэтому я подумал, что используя эти данные можно будет
компенсировать провалы напряжения и отдать в телеметрию ровный график
разряда. Конечно, внутреннее сопротивление разных батареек различно,
но его можно оценить по току и просадке напряжения при начальной
раскрутке ротора.

  Таким образом, надо было выбрать микроконтроллер и написать
прошивку, которая бы анализировала данные jlog, обрабатывала их и
передавала бы в sbus2.  При этом, по возможности обойтись готовыми
компонентами, которые легко приобрести.  Задача осложнилась тем, что в
sbus используется инвертированный uart и мало микрокотроллеров такое
умеет, а внешний инвертор делать не хотелось.  Выбор в итоге пал на
довольно популярную плату на базе stm32f103c8t6.  У этого чипа нет
инвертора uart, но eго производительности хватает чтобы сделать
програмную эмуляцию.

## Результат

  После написания прошивки получился следющий результат: ![график](https://raw.githubusercontent.com/delamonpansie/jitel/master/img/graph.png)

Это телеметрия напряжения с одного полета, которая писалась
аппаратурой. Красным цветом сырые данные, зеленым отфильтрованные.
Алярм, естественно, настроен на зеленые данные.


## Сборка

Для сборки понадобятся:
 * [плата на базе stm32f103c8t6](http://ru.aliexpress.com/item/1pcs-STM32F103C8T6-ARM-STM32-Minimum-System-Development-Board-Module-For-arduino/32583160323.html?spm=2114.30010708.3.2.SeRtii&ws_ab_test=searchweb201556_7,searchweb201602_4_10039_10048_10057_10047_10056_10037_10055_10049_10059_10033_10046_10058_10032_10045_10017_10060_10061_10062_10063_412_10064,searchweb201603_7&btsid=9cf5342e-03d1-481e-ba5a-e0c08071b91a)
   ![board](https://raw.githubusercontent.com/delamonpansie/jitel/master/img/board.jpg)
 * [stlink v2](http://ru.aliexpress.com/item/Hot-Sale-ST-LINK-Stlink-ST-Link-V2-Mini-STM8-STM32-Simulator-Download-Programmer-Programming-With/32684040486.html?spm=2114.30010708.3.2.yQkglt&ws_ab_test=searchweb201556_7,searchweb201602_4_10039_10048_10057_10047_10056_10037_10055_10049_10059_10033_10046_10058_10032_10045_10017_10060_10061_10062_10063_412_10064,searchweb201603_7&btsid=4ed7ac67-42da-4f93-981b-848073d270a8)
   ![st link](https://raw.githubusercontent.com/delamonpansie/jitel/master/img/st-link.jpg)
 * резистор на 330-560 Ом

## Схема
   ![схема](https://raw.githubusercontent.com/delamonpansie/jitel/master/img/schematic.jpg)

## Сборка прошивки

Перед тем, как прошивать плату надо запустить в отдельном терминале свежий (0.10) openocd:
   `openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg`

```sh
$ git clone http://github.com/delamonpansie/jitel
$ git submodule update
$ git submodule init
$ make -C libopencm3
$ make
$ make flash
```

## Слоты
  * 1-3: Curr-1678, сырые данные jive
  * 4: SBS-1RM, обороты. Для корреткной работы надо выборать тип "magnetic" и выставить gear rate
  * 5-6: компенсированное значение напряжения


## Некоторые детали устройства

  Описание протокола jive: настройки для порта обычные, 9800,8n1.
Посылки идут каждые 100ms и состоят из 25 16-битных значений.  Первые
четыре байта magic, далее идут параметры, часть из которых передается
в каждом фрейме, часть с периодическими паузами. Я сумел понять где
передаются: ток и напряжение bec, ток и напряжения силовой батарейки,
обрототы на моторе. Подозреваю, что магический константа для оборотов
свзязаны с кол-вом полюсов мотора, но доказать не могу. Мне не удалось
понять, где передается температура, но, честно говоря, не очень-то и
старался. Детальное описание структуры есть в исходниках.

  В детальном описании sbus2 не вижу смысла: желающие могут
воспользоваться следующими ссылкой: https://sites.google.com/site/sbus2diy/home

  Устройство программного инвертора следующее: вход от sbus подается на
ногу PB4; на PB4 висит EXTI прерывание, которое по изменению уровня
дергает PA4 в инверсии; PA4 закорочено с PA3, которое является входом
от USART2. В обратную сторону аналогично.
