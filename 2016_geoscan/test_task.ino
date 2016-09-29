/*
  Arduino project file for solution lab's test task

  Code for solar tracker using user's coordinates (latitude/longitude), as well as current date and time.
  Information can be displayed onto lcd (2x16), by pressing the button.
  Platform is using three stoppers to control stepper's errors (2 vertical, 1 horizontal)

  Created 27.09.2016
  By Azarn

  https://github.com/Azarn/SolutionLab/2016-geoscan/test_task.ino

*/

#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>
#include <Stepper.h>

// #define DEBUG_CALC

const int LCD_SHOWUP_MS = 5000;                 // Время показа сообщения на экран
const int STEPS = 200;                          // Количество шагов двигателя
const double STEPPER_DEGREE_VERT = 1.8 / 3.0;   // Угол поворота по горизонтали на один шаг с поправкой на редуктор
const double STEPPER_DEGREE_HORIZ = 1.8 / 3.0;  // Угол поворота по вертикали на один шаг с поправкой на редуктор
const double EPOCH_JD = 2451545.0;              // Юлианская дата для 1 января 2000 года
const double SUN_H_THRESHOLD = 6.0;             // Ограничение угла трекинга солнца по высоте
const double ANGLE_CORRECTION_VERT = -3.0;      // Коррецкия угла по вертикали
const double ANGLE_CORRECTION_HORIZ = 6.0;      // Коррекция угла по горизонтали

double my_lat, my_lng;                             // Широта и долгота потребителя
int gmt;                                           // Часовой пояс
int current_step_horiz = 0, current_step_vert = 0; // Счетчик количества пройденных шагов 
unsigned long whenShowingData = 0;                 // Хранит время нажатия на кнопку
bool showingData = false;                          // Флаг показа данных на экране
bool onStartPosition = false;                      // Флаг нахождения в стартовой позиции

// Используемые пины
int btn_mode_pin = 3;                              
int stopper_horiz_pin = A0;
int stopper_vert_bottom_pin = A1;
int stopper_vert_top_pin = A2;
int horiz_stepper_pins[] = {5, 6, 7, 8};
int vert_steper_pins[] = {9, 10, 11, 12};

LiquidCrystal_I2C lcd(0x27, 16, 2);
Stepper horiz_stepper(STEPS, horiz_stepper_pins[0], horiz_stepper_pins[1], horiz_stepper_pins[2], horiz_stepper_pins[3]);
Stepper vert_stepper(STEPS, vert_steper_pins[0], vert_steper_pins[1], vert_steper_pins[2], vert_steper_pins[3]);


/*
 * Возвращает нормализованный угол в радианах (от -pi до pi)
 */
inline double norm_rad(double a) {
  return atan2(sin(a), cos(a));
}

/*
 * Функция для расчёта значения гипотенузы
 */
inline double hypot(double x, double y) {
  return sqrt(x * x + y * y);
}

/*
 * Возвращает текущее местное время
 */
tmElements_t getLocalTM() {
  tmElements_t tm;
  breakTime(now(), tm);
  return tm;
}

/*
 * Возвращает текущее время в формате UTC
 */
tmElements_t getUTC_TM() {
  tmElements_t tm;
  time_t tt = now();
  tt += 3600 * -gmt;
  breakTime(tt, tm);
  return tm;
}

/*
 * Возвращает количество дней, прошедших с 1 января 2000 года
 */
double compute_jd(tmElements_t const &tm) {
  int month = tm.Month;
  int year = tm.Year + 1970;
  if (month <= 2) {
    month += 12;
    --year;
  }
  return ((long)(365.25 * year) + (long)(30.6001 * (month + 1)) - 15 + 
          (1720996.5 - EPOCH_JD) + tm.Day + tm.Hour / 24.0 + 
          tm.Minute / (24.0 * 60) + tm.Second / (24.0 * 60 * 60));    
}

/*
 * Вычисляет среднюю и истинную долготу солнца
 */
void compute_solar_longitude(double t, double &l, double &l0) {
  // Средняя аномалия
  double M = radians(fmod(357.52910 + 35999.05030 * t - 0.0001559 * t * t, 360));

  // Средняя долгота (в градусах)
  l0 = fmod(280.46645 + 36000.76983 * t + 0.0003032 * t * t, 360);
  double DL = (1.914600 - 0.004817 * t - 0.000014 * t * t) * sin(M) + 
              (0.019993 - 0.000101 * t) * sin(2 * M) + 0.000290 * sin(3 * M);

  l = radians(l0 + DL);
  l0 = radians(l0);
}

/*
 * Вычисляет склонение и прямое восхождение солнца
 */
void compute_delta_and_ra(double l, double jd, double &decl, double &ra) {
  // Наклон оси вращения
  double eps = radians(23.43929111 - 3.563E-7 * jd);
    
  double x = cos(l);
  double y = sin(l) * cos(eps);
  double z = sin(l) * sin(eps);

  decl = atan2(z, hypot(x, y));
  ra = atan2(y, x);
 }

/*
 * Возвращает местное звёздное время
 */
inline double get_sidereal_time_at_lng(double l0, double lng, tmElements_t const &tm) {
    return l0 + PI + radians((tm.Hour +
                             tm.Minute / 60.0 + tm.Second / 3600.0) * 15) + lng;
}

/*
 * Вычисляет высоту и азимут солнца
 */
void compute_altitude_and_azimuth(double &h, double &az) {
  // Щепотка магии
  tmElements_t tm = getUTC_TM(); 
  double jd = compute_jd(tm);
  double t = jd / 36525;
  double l, l0;
  compute_solar_longitude(t, l, l0);
  double delta, ra;
  compute_delta_and_ra(l, jd, delta, ra);
  double theta = get_sidereal_time_at_lng(l0, radians(my_lng), tm);
  double tau = norm_rad(theta - ra);
  
  double x = cos(tau) * cos(delta);
  double z = sin(delta);

  double lat = radians(my_lat);
  double xhor = x * sin(lat) - z * cos(lat);
  double yhor = sin(tau) * cos(delta);
  double zhor = x * cos(lat) + z * sin(lat);

  h = atan2(zhor, hypot(xhor, yhor));
  az = atan2(yhor, xhor);

  #ifdef DEBUG_CALC             // Используется для отладки вычисленных значений
  Serial.println(jd, 6);
  Serial.println(t, 6);
  Serial.println(degrees(l), 6);
  Serial.println(degrees(delta), 6);
  Serial.println(degrees(ra), 6);
  Serial.println(fmod(degrees(theta), 360), 6);
  Serial.println(degrees(tau), 6);
  Serial.println(degrees(h), 6);
  Serial.println(degrees(az), 6);
  Serial.println();
  #endif
}

/*
 * Выводит блок данных (дата/время + высота и азимут) на экран в две строки)
 */
void printData(double h, double az) {
  tmElements_t tm = getLocalTM();
  String date_str = String(tm.Day);
  date_str += "/";
  date_str += String(tm.Month);
  date_str += "/";
  date_str += String(tm.Year + 1970);
  date_str += " ";
  date_str += String(tm.Hour);
  date_str += String(":");
  if (tm.Minute < 10) {
    date_str += "0";
  }
  date_str += String(tm.Minute);
  
  lcd.setCursor(0, 0);
  lcd.print(date_str);

  lcd.setCursor(0, 1);
  lcd.print(String(h, 3) + " " + String(az, 3));
}

/*
 * Двигаемся в начальное положение до стопоров
 */
void moveToStart() {
  while(digitalRead(stopper_horiz_pin) != HIGH) {
    horiz_stepper.step(-1);
  }

  while(digitalRead(stopper_vert_bottom_pin) != HIGH) {
    vert_stepper.step(-1);
  }
  onStartPosition = true;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Setup started");
  
  lcd.init();
  lcd.noDisplay();

  // В силу отсутствия блока управления, задаём вручную время и локальные координаты
  my_lat = 59.9577927;
  my_lng = 30.4167652;
  gmt = 4;
  setTime(7, 9, 0, 29, 9, 2016);

  pinMode(btn_mode_pin, INPUT);
  pinMode(stopper_horiz_pin, INPUT);
  pinMode(stopper_vert_bottom_pin, INPUT);
  pinMode(stopper_vert_top_pin, INPUT);

  horiz_stepper.setSpeed(15);
  vert_stepper.setSpeed(15);

  moveToStart();

  Serial.println("Setup completed");
}

void loop() {
  double alt, az;
  compute_altitude_and_azimuth(alt, az);
  alt = degrees(alt) - ANGLE_CORRECTION_VERT;            // Поправка на угол отклонения самой конструкции
  az = degrees(az + PI / 2); - ANGLE_CORRECTION_HORIZ;   // Доворачиваем, чтобы смотреть на восток (+ поправка)

  if (alt > SUN_H_THRESHOLD) {      // Проверяем достаточно ли высоко встало солнце
    int v_steps = (int)(alt / STEPPER_DEGREE_VERT) - current_step_vert;     // Разница в шагах по вертикали
    if ((v_steps >= 1 && digitalRead(stopper_vert_top_pin) == LOW) ||       // Доворот вверх и НЕ в верхний стопор
        (v_steps <= -1 && digitalRead(stopper_vert_bottom_pin) == LOW)) {   // Доворот вниз и НЕ в нижний стопор
      v_steps = v_steps > 0 ? 1 : -1;     // В целях безопасности вертимся только на один шаг
      vert_stepper.step(v_steps);
      current_step_vert += v_steps;
      Serial.print("Changing vertial step to: ");
      Serial.println(current_step_vert);
    }

    int h_steps = (int)(az / STEPPER_DEGREE_HORIZ) - current_step_horiz;  // Разница в шагах по горизонтали
    if (h_steps != 0 && digitalRead(stopper_horiz_pin == LOW)) {  // Разрешаем поворот, только если не уперлись в стопор
      h_steps = h_steps > 0 ? 1 : -1;
      horiz_stepper.step(h_steps);
      current_step_horiz += h_steps;
      Serial.print("Changing horizontal step to: ");
      Serial.println(current_step_horiz);
    }
  } else if (!onStartPosition) {
    moveToStart();      // Если солнце зашло, то можно спокойно вернуться в начальное положение
  }

  /*
   * Блок отвечает за показ даты/времени и текущих значений высоты и азимута при нажатии на кнопку
   */
  if (digitalRead(btn_mode_pin) == HIGH) {
    if (!showingData) {
      lcd.display();
      lcd.backlight();
      lcd.clear();
    }
    showingData = true;           // Разрешаем показ данных на экране
    whenShowingData = millis();   // Запоминаем, когда нажали на кнопку
  }

  if (showingData) {
    printData(alt, az);
    if (millis() - whenShowingData > LCD_SHOWUP_MS) {   // Сработает через LCD_SHOWUP_MS мс после нажатия
      showingData = false;
      lcd.noBacklight();
      lcd.noDisplay();
    }
  }
}
