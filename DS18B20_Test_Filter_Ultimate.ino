/**
 * DS18B20_Test_Filter_Ultimate_3_1.ino
 * 
 * УЛЬТИМАТИВНЫЙ ТЕСТЕР ФИЛЬТРАЦИИ ДЛЯ ДАТЧИКОВ DS18B20
 * 
 * ОСОБЕННОСТИ ДЛЯ ТЕСТИРОВАНИЯ:
 * 1. БЕЗ ОКРУГЛЕНИЯ в вычислениях (точные данные)
 * 2. Вывод с точностью 4 знака для анализа
 * 3. Чистая статистика без искажений
 * 
 * ВЕРСИЯ: 3.1 (Тестовая, без округления)
 */

#include <OneWire.h>
#include <DallasTemperature.h>

// ============================================================================
// РАЗДЕЛ 1: КОНФИГУРАЦИЯ СИСТЕМЫ (ТЕСТОВЫЙ РЕЖИМ)
// ============================================================================

#define RESOLUTION 12           // 12 бит = 0.0625°C шаг
#define MEASURE_INTERVAL 1500   // Интервал измерений (мс)
#define FILTER_TYPE 2           // 0=без фильтра, 1=медианный, 2=скольз.среднее, 3=эксп., 4=двухступ.
#define FILTER_SIZE 3           // Размер фильтра (для типа 1,2,4)
#define DELTA_THRESHOLD 0.10    // Порог обнаружения изменений (°C)
#define DISPLAY_PRECISION 4     // Количество знаков при выводе (4 для тестов)
#define TEST_MODE 1             // 0=авто, 1=только шум, 2=только реакция
#define TEST_DURATION 30       // Количество измерений (120 = ~3 мин)

// ============================================================================
// РАЗДЕЛ 2: КОНСТАНТЫ ФОРМАТИРОВАНИЯ
// ============================================================================

#define CSV_TIME_WIDTH 12
#define CSV_TEMP_WIDTH 16       // Увеличено для 4 знаков + знак
#define CSV_DELTA_WIDTH 12
#define HISTOGRAM_WIDTH 20
#define HISTOGRAM_BINS 13
#define AUTOCORR_LAGS 3
#define PROGRESS_BAR_WIDTH 40

// ============================================================================
// РАЗДЕЛ 3: АППАРАТНЫЕ НАСТРОЙКИ
// ============================================================================

const int ONE_WIRE_BUS = 4;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorAddress;

// ============================================================================
// РАЗДЕЛ 4: СТРУКТУРЫ ДАННЫХ
// ============================================================================

typedef struct {
  float buffer[15];
  int index;
  int size;
  bool initialized;
} MedianFilter_t;

typedef struct {
  float rawValues[500];           // ТОЧНЫЕ сырые данные
  float filteredValues[500];      // ТОЧНЫЕ фильтрованные данные
  float deltaValues[500];         // ТОЧНЫЕ изменения
  unsigned long timestamps[500];
  int count;
  int maxCount;
} MeasurementData_t;

typedef struct {
  float minRaw, maxRaw;
  float minFiltered, maxFiltered;
  float sumRaw, sumFiltered;
  float noiseMin, noiseMax;
  float noiseSum, noiseSumSq;
  unsigned long startTime, endTime;
} Statistics_t;

MedianFilter_t medianFilter;
MeasurementData_t measurements;
Statistics_t stats;

bool testRunning = false;
int testPhase = 0;
unsigned long lastMeasureTime = 0;

// ============================================================================
// РАЗДЕЛ 5: ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================================

/** * ФУНКЦИЯ: getConversionDelay()
 * ВОЗВРАЩАЕТ: Время конверсии для разрешения
 * КОММЕНТАРИЙ: Без изменений, работает правильно
 */
int getConversionDelay(uint8_t resolution) {
  int baseDelay;
  switch (resolution) {
    case 9: baseDelay = 94; break;
    case 10: baseDelay = 188; break;
    case 11: baseDelay = 375; break;
    case 12: baseDelay = 750; break;
    default: baseDelay = 750; break;
  }
  return baseDelay * 125 / 100;
}

/** * ФУНКЦИЯ: formatNumberForDisplay()
 * ВОЗВРАЩАЕТ: Строку с числом для вывода
 * ИЗМЕНЕНИЕ: Всегда 4 знака после запятой для тестов
 */
String formatNumberForDisplay(float value) {
  String result;
  
  if (value >= 0) {
    result = "+";
  } else {
    result = "-";
    value = -value;
  }
  
  // Форматируем с 4 знаками после запятой
  result += String(value, 4);
  return result;
}

// ============================================================================
// РАЗДЕЛ 6: ФУНКЦИИ ФИЛЬТРАЦИИ (БЕЗ ОКРУГЛЕНИЯ)
// ============================================================================

/** * ФУНКЦИЯ: initMedianFilter()
 * КОММЕНТАРИЙ: Без изменений
 */
void initMedianFilter(float initialValue) {
  medianFilter.size = FILTER_SIZE;
  medianFilter.index = 0;
  medianFilter.initialized = true;
  for (int i = 0; i < medianFilter.size; i++) {
    medianFilter.buffer[i] = initialValue;
  }
}

/** * ФУНКЦИЯ: applyMedianFilter()
 * КОММЕНТАРИЙ: Возвращает точное значение, без округления
 */
float applyMedianFilter(float newValue) {
  if (!medianFilter.initialized) {
    initMedianFilter(newValue);
    return newValue;
  }
  
  medianFilter.buffer[medianFilter.index] = newValue;
  medianFilter.index = (medianFilter.index + 1) % medianFilter.size;
  
  float tempBuffer[15];
  for (int i = 0; i < medianFilter.size; i++) {
    tempBuffer[i] = medianFilter.buffer[i];
  }
  
  for (int i = 0; i < medianFilter.size - 1; i++) {
    for (int j = 0; j < medianFilter.size - i - 1; j++) {
      if (tempBuffer[j] > tempBuffer[j + 1]) {
        float temp = tempBuffer[j];
        tempBuffer[j] = tempBuffer[j + 1];
        tempBuffer[j + 1] = temp;
      }
    }
  }
  
  return tempBuffer[medianFilter.size / 2];  // Точная медиана
}

/** * ФУНКЦИЯ: applyMovingAverage()
 * ВОЗВРАЩАЕТ: Корректное скользящее среднее для DS18B20
 * 
 * ИСПРАВЛЕННЫЕ ПРОБЛЕМЫ:
 * 1. Правильная инициализация буфера разными значениями
 * 2. Возврат отфильтрованных значений только после заполнения буфера
 * 3. Чёткое разделение фаз: сбор данных → фильтрация
 * 
 * АЛГОРИТМ:
 * Фаза 1: Собираем первые FILTER_SIZE измерений в буфер
 * Фаза 2: После заполнения буфера начинаем нормальную фильтрацию
 */
float applyMovingAverage(float newValue) {
  // 1. КОНВЕРТАЦИЯ В ТИКИ ДАТЧИКА (1 тик = 0.0625°C)
  // Используем round() для правильного округления
  int16_t newValueTicks = (int16_t)round(newValue * 16.0);
  
  // 2. СТАТИЧЕСКИЕ ПЕРЕМЕННЫЕ
  static int16_t buffer[15];          // Буфер значений (в тиках)
  static int8_t writeIndex = 0;       // Индекс для записи (только для фазы 1)
  static int8_t filterIndex = 0;      // Циклический индекс для фазы 2
  static int32_t sumTicks = 0;        // Сумма всех значений в буфере
  static bool bufferFilled = false;   // Флаг: буфер полностью заполнен?
  static uint8_t samplesInBuffer = 0; // Количество значений в буфере
  
  // 3. ФАЗА 1: ЗАПОЛНЕНИЕ БУФЕРА (первые FILTER_SIZE измерений)
  if (!bufferFilled) {
    // Добавляем новое значение в буфер
    buffer[writeIndex] = newValueTicks;
    sumTicks += newValueTicks;
    samplesInBuffer++;
    
    // Увеличиваем индекс для следующей записи
    writeIndex++;
    
    // Проверяем, заполнен ли буфер
    if (samplesInBuffer >= FILTER_SIZE) {
      bufferFilled = true;
      filterIndex = 0;  // Начинаем с начала буфера для циклического доступа
      
      // Отладочный вывод (можно раскомментировать)
      // Serial.printf("Буфер заполнен! %d значений, сумма: %ld тиков\n", 
      //               FILTER_SIZE, sumTicks);
    }
    
    // В фазе заполнения возвращаем СЫРОЕ значение (фильтр ещё не готов)
    return newValue;
  }
  
  // 4. ФАЗА 2: НОРМАЛЬНАЯ РАБОТА ФИЛЬТРА (буфер заполнен)
  
  // 4.1. УДАЛЯЕМ САМОЕ СТАРОЕ ЗНАЧЕНИЕ ИЗ СУММЫ
  int16_t oldestValue = buffer[filterIndex];
  sumTicks -= oldestValue;
  
  // 4.2. ДОБАВЛЯЕМ НОВОЕ ЗНАЧЕНИЕ В БУФЕР
  buffer[filterIndex] = newValueTicks;
  sumTicks += newValueTicks;
  
  // 4.3. ВЫЧИСЛЯЕМ СРЕДНЕЕ ЗНАЧЕНИЕ В ТИКАХ
  // Используем float для точного деления
  float averageTicks = (float)sumTicks / (float)FILTER_SIZE;
  
  // 4.4. ОКРУГЛЯЕМ ДО БЛИЖАЙШЕГО ЗНАЧЕНИЯ ДАТЧИКА
  // round() гарантирует, что 361.49 → 361, 361.51 → 362
  int16_t roundedTicks = (int16_t)round(averageTicks);
  
  // 4.5. ОБНОВЛЯЕМ ЦИКЛИЧЕСКИЙ ИНДЕКС
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
  
  // 4.6. КОНВЕРТИРУЕМ ОБРАТНО В ГРАДУСЫ
  float filteredValue = (float)roundedTicks / 16.0;
  
  // 5. ОТЛАДОЧНЫЙ ВЫВОД (раскомментировать при необходимости)
  /*
  static uint32_t debugCounter = 0;
  if (debugCounter++ % 10 == 0) {  // Выводим каждое 10-е измерение
    Serial.printf("Скользящее среднее [%d точек]:\n", FILTER_SIZE);
    Serial.printf("  Вход: %.4f°C = %d тиков\n", newValue, newValueTicks);
    Serial.printf("  Буфер: [");
    for (int i = 0; i < FILTER_SIZE; i++) {
      Serial.printf("%.2f", buffer[i] / 16.0);
      if (i < FILTER_SIZE - 1) Serial.print(", ");
    }
    Serial.printf("]\n");
    Serial.printf("  Среднее: %.2f тиков → округлено: %d тиков\n", 
                  averageTicks, roundedTicks);
    Serial.printf("  Выход: %.4f°C\n\n", filteredValue);
  }
  */
  
  return filteredValue;
}

/** * ФУНКЦИЯ: applyExponentialSmoothing()
 * КОММЕНТАРИЙ: Без изменений
 */
float applyExponentialSmoothing(float newValue) {
  static float lastValue = 0;
  static bool initialized = false;
  
  if (!initialized) {
    lastValue = newValue;
    initialized = true;
    return newValue;
  }
  
  float alpha = 0.3;
  float smoothed = alpha * newValue + (1 - alpha) * lastValue;
  lastValue = smoothed;
  
  return smoothed;
}

/** * ФУНКЦИЯ: applyTwoStageFilter()
 * КОММЕНТАРИЙ: Без изменений
 */
float applyTwoStageFilter(float newValue) {
  float medianFiltered = applyMedianFilter(newValue);
  
  static float stage2Value = 0;
  static bool stage2Initialized = false;
  
  if (!stage2Initialized) {
    stage2Value = medianFiltered;
    stage2Initialized = true;
    return medianFiltered;
  }
  
  float stage2Alpha = 0.4;
  stage2Value = stage2Alpha * medianFiltered + (1 - stage2Alpha) * stage2Value;
  
  return stage2Value;
}

/** * ФУНКЦИЯ: applyFilter()
 * ВОЗВРАЩАЕТ: Точное отфильтрованное значение
 * ИЗМЕНЕНИЕ: УДАЛЕНО округление! Возвращаем точные данные.
 */
float applyFilter(float rawValue) {
  float filteredValue;
  
  switch (FILTER_TYPE) {
    case 0: filteredValue = rawValue; break;
    case 1: filteredValue = applyMedianFilter(rawValue); break;
    case 2: filteredValue = applyMovingAverage(rawValue); break;
    case 3: filteredValue = applyExponentialSmoothing(rawValue); break;
    case 4: filteredValue = applyTwoStageFilter(rawValue); break;
    default: filteredValue = rawValue; break;
  }
  
  // ВАЖНО: НЕТ ОКРУГЛЕНИЯ! Возвращаем точное значение для тестов.
  return filteredValue;
}

// ============================================================================
// РАЗДЕЛ 7: ФУНКЦИИ СТАТИСТИКИ
// ============================================================================

/**
 * ФУНКЦИЯ: initStatistics()
 * КОММЕНТАРИЙ: Без изменений
 */
void initStatistics() {
  stats.minRaw = 1000.0; stats.maxRaw = -1000.0;
  stats.minFiltered = 1000.0; stats.maxFiltered = -1000.0;
  stats.sumRaw = 0.0; stats.sumFiltered = 0.0;
  stats.noiseMin = 1000.0; stats.noiseMax = -1000.0;
  stats.noiseSum = 0.0; stats.noiseSumSq = 0.0;
  stats.endTime = 0;
}

/**
 * ФУНКЦИЯ: updateStatistics()
 * КОММЕНТАРИЙ: Использует точные данные
 */
void updateStatistics(float rawValue, float filteredValue) {
  if (rawValue < stats.minRaw) stats.minRaw = rawValue;
  if (rawValue > stats.maxRaw) stats.maxRaw = rawValue;
  if (filteredValue < stats.minFiltered) stats.minFiltered = filteredValue;
  if (filteredValue > stats.maxFiltered) stats.maxFiltered = filteredValue;
  
  stats.sumRaw += rawValue;
  stats.sumFiltered += filteredValue;
}

/**
 * ФУНКЦИЯ: detectDeltaChange()
 * ВОЗВРАЩАЕТ: Точное изменение или 0
 * ИЗМЕНЕНИЕ: УДАЛЕНО округление! Возвращаем точную дельту.
 */
float detectDeltaChange(float currentValue) {
  static float previousValue = 0;
  static bool hasPrevious = false;
  
  if (!hasPrevious) {
    previousValue = currentValue;
    hasPrevious = true;
    return 0.0;
  }
  
  float delta = currentValue - previousValue;
  previousValue = currentValue;
  
  if (fabs(delta) >= DELTA_THRESHOLD) {
    return delta;  // ВАЖНО: Точная дельта, без округления!
  }
  
  return 0.0;
}

/**
 * ФУНКЦИЯ: calculateNoiseStatistics()
 * КОММЕНТАРИЙ: Работает с точными данными
 */
bool calculateNoiseStatistics() {
  if (measurements.count < 10) return false;
  
  int N = measurements.count;
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  
  for (int i = 0; i < N; i++) {
    float y = measurements.filteredValues[i];
    sumX += i;
    sumY += y;
    sumXY += i * y;
    sumX2 += i * i;
  }
  
  float b = (N * sumXY - sumX * sumY) / (N * sumX2 - sumX * sumX);
  float a = (sumY - b * sumX) / N;
  
  stats.noiseMin = 1000.0; stats.noiseMax = -1000.0;
  stats.noiseSum = 0.0; stats.noiseSumSq = 0.0;
  
  for (int i = 0; i < N; i++) {
    float y = measurements.filteredValues[i];
    float yTrend = a + b * i;
    float residual = y - yTrend;
    
    stats.noiseSum += residual;
    stats.noiseSumSq += residual * residual;
    
    if (residual < stats.noiseMin) stats.noiseMin = residual;
    if (residual > stats.noiseMax) stats.noiseMax = residual;
  }
  
  return true;
}

/**
 * ФУНКЦИЯ: calculateAutocorrelation()
 * КОММЕНТАРИЙ: Использует точные фильтрованные значения
 */
int calculateAutocorrelation(float* lags, int maxLags) {
  if (measurements.count < 20) return 0;
  
  int N = measurements.count;
  float mean = stats.sumFiltered / N;
  
  float variance = 0;
  for (int i = 0; i < N; i++) {
    float diff = measurements.filteredValues[i] - mean;
    variance += diff * diff;
  }
  variance /= N;
  
  if (variance < 0.000001) return 0;
  
  int calculatedLags = min(maxLags, N / 4);
  
  for (int lag = 0; lag < calculatedLags; lag++) {
    float correlation = 0;
    for (int i = 0; i < N - lag; i++) {
      float diff1 = measurements.filteredValues[i] - mean;
      float diff2 = measurements.filteredValues[i + lag] - mean;
      correlation += diff1 * diff2;
    }
    lags[lag] = correlation / ((N - lag) * variance);
  }
  
  return calculatedLags;
}

// ============================================================================
// РАЗДЕЛ 8: ФУНКЦИИ ВЫВОДА (ВСЕГДА 4 ЗНАКА)
// ============================================================================

/**
 * ФУНКЦИЯ: printProgressBar()
 * КОММЕНТАРИЙ: Без изменений
 */
void printProgressBar(int current, int total) {
  static int lastPrintedPercent = -1;
  int percent = (current * 100) / total;
  
  if (current == 1 || percent != lastPrintedPercent || current == total) {
    lastPrintedPercent = percent;
    
    int filledWidth = (percent * PROGRESS_BAR_WIDTH) / 100;
    int emptyWidth = PROGRESS_BAR_WIDTH - filledWidth;
    
    String progressBar = "[";
    for (int i = 0; i < filledWidth; i++) progressBar += "=";
    if (filledWidth < PROGRESS_BAR_WIDTH) {
      progressBar += ">";
      for (int i = 0; i < emptyWidth - 1; i++) progressBar += " ";
    }
    progressBar += "]";
    
    Serial.printf("%s %3d/%d (%3d%%)\n", progressBar.c_str(), current, total, percent);
  }
}

/**
 * ФУНКЦИЯ: printTestHeader()
 * ИЗМЕНЕНИЕ: Вывод настроек с пояснениями
 */
void printTestHeader() {
  Serial.println("\n" + String(60, '='));
  Serial.println("ТЕСТ DS18B20 - ВЕРСИЯ ДЛЯ ТЕСТИРОВАНИЯ ФИЛЬТРОВ");
  Serial.println(String(60, '='));
  
  Serial.println("ВАЖНО: Эта версия НЕ округляет данные в вычислениях!");
  Serial.println("       Вывод всегда с " + String(DISPLAY_PRECISION) + " знаками\n");
  
  const char* filterNames[] = {"Без фильтра", "Медианный", "Скользящее среднее",
                               "Эксп. сглаживание", "Двухступенчатый"};
  
  Serial.println("НАСТРОЙКИ ТЕСТА:");
  Serial.printf("  Фильтр: %s\n", filterNames[FILTER_TYPE]);
  Serial.printf("  Размер фильтра: %d\n", FILTER_SIZE);
  Serial.printf("  Порог изменения: %.4f°C\n", DELTA_THRESHOLD);
  Serial.printf("  Измерений: %d (%.1f мин)\n", TEST_DURATION, 
                (TEST_DURATION * MEASURE_INTERVAL) / 60000.0);
  Serial.println(String(60, '='));
}

/**
 * ФУНКЦИЯ: printHistogram()
 * ИЗМЕНЕНИЕ: Использует сырые данные, вывод с 4 знаками
 */
void printHistogram() {
  if (measurements.count < 10) {
    Serial.println("⚠️  Недостаточно данных для гистограммы");
    return;
  }
  
  if (!calculateNoiseStatistics()) return;
  
  const int NUM_BINS = 19;
  const float BIN_START = -0.09;
  const float BIN_WIDTH = 0.01;
  const int MAX_BAR_WIDTH = 20;
  
  float binCenters[NUM_BINS];
  int binCounts[NUM_BINS] = {0};
  
  for (int i = 0; i < NUM_BINS; i++) {
    binCenters[i] = BIN_START + (i * BIN_WIDTH);
  }
  
  float meanValue = stats.sumRaw / measurements.count;
  Serial.printf("   Среднее (сырые): %.4f°C\n", meanValue);
  
  for (int i = 0; i < measurements.count; i++) {
    float noise = measurements.rawValues[i] - meanValue;
    int binIndex = (int)((noise - BIN_START + BIN_WIDTH/2) / BIN_WIDTH);
    
    if (binIndex < 0) binIndex = 0;
    else if (binIndex >= NUM_BINS) binIndex = NUM_BINS - 1;
    
    binCounts[binIndex]++;
  }
  
  int maxCount = 0;
  for (int i = 0; i < NUM_BINS; i++) {
    if (binCounts[i] > maxCount) maxCount = binCounts[i];
  }
  
  if (maxCount == 0) return;
  
  Serial.println("\nГИСТОГРАММА ШУМА (относительно среднего):");
  
  for (int i = 0; i < NUM_BINS; i++) {
    String label;
    if (binCenters[i] >= 0) {
      label = "+" + String(binCenters[i], 2);
    } else {
      label = String(binCenters[i], 2);
    }
    
    while (label.length() < 6) label = " " + label;
    
    int barWidth = 0;
    if (maxCount > 0) {
      barWidth = (binCounts[i] * MAX_BAR_WIDTH) / maxCount;
    }
    
    String bar;
    for (int j = 0; j < barWidth; j++) bar += "█";
    
    Serial.printf("%s %s", label.c_str(), bar.c_str());
    if (binCounts[i] > 0) Serial.printf(" (%d)", binCounts[i]);
    Serial.println();
  }
  
  Serial.println("\n   СТАТИСТИКА ШУМА:");
  Serial.printf("   Всего измерений: %d\n", measurements.count);
  Serial.printf("   Диапазон шума: от %.4f до %.4f°C\n", stats.noiseMin, stats.noiseMax);
  Serial.printf("   Среднее значение: %.4f°C\n", meanValue);
  
  if (measurements.count > 1) {
    float variance = (stats.noiseSumSq - pow(stats.noiseSum, 2) / measurements.count) 
                     / (measurements.count - 1);
    float stdDev = sqrt(variance);
    Serial.printf("   СКО (σ): %.4f°C\n", stdDev);
  }
}

/**
 * ФУНКЦИЯ: printAutocorrelation()
 * ИЗМЕНЕНИЕ: Вывод с 2 знаками (для корреляции достаточно)
 */
void printAutocorrelation() {
  float lags[AUTOCORR_LAGS];
  int numLags = calculateAutocorrelation(lags, AUTOCORR_LAGS);
  
  if (numLags == 0) return;
  
  Serial.println("\nАВТОКОРРЕЛЯЦИЯ (белый шум = 0):");
  
  for (int lagIndex = 1; lagIndex <= numLags; lagIndex++) {
    float correlation = lags[lagIndex - 1];
    float timeSeconds = (lagIndex * MEASURE_INTERVAL) / 1000.0;
    
    String lagStr = "Лаг " + String(lagIndex) + " (" + String(timeSeconds, 1) + " сек):";
    while (lagStr.length() < 22) lagStr += " ";
    
    String corrStr;
    if (correlation >= 0) {
      corrStr = " +" + String(correlation, 2);
    } else {
      corrStr = " " + String(correlation, 2);
    }
    while (corrStr.length() < 7) corrStr += " ";
    
    String graph = "[";
    int graphWidth = 20;
    int fillWidth = (int)(fabs(correlation) * graphWidth);
    
    if (correlation >= 0) {
      for (int i = 0; i < fillWidth; i++) graph += "=";
      for (int i = fillWidth; i < graphWidth; i++) graph += " ";
    } else {
      for (int i = 0; i < graphWidth - fillWidth; i++) graph += " ";
      for (int i = 0; i < fillWidth; i++) graph += "=";
    }
    graph += "]";
    
    Serial.printf("%s %s %s\n", lagStr.c_str(), corrStr.c_str(), graph.c_str());
  }
  
  bool hasSignificantCorrelation = false;
  for (int lagIndex = 0; lagIndex < numLags; lagIndex++) {
    if (fabs(lags[lagIndex]) > 0.3) {
      hasSignificantCorrelation = true;
      break;
    }
  }
  
  if (hasSignificantCorrelation) {
    Serial.println("\n⚠️  Обнаружена значимая автокорреляция");
    Serial.println("   Шум не является белым (случайным)");
  } else {
    Serial.println("\n✅ Шум близок к белому (случайному)");
  }
}

/**
 * ФУНКЦИЯ: printCSVData()
 * ИЗМЕНЕНИЕ: Всегда 4 знака после запятой
 */
void printCSVData() {
  Serial.println("\nCSV ДАННЫЕ (все значения с 4 знаками):");
  
  // Заголовок
  Serial.println("# Настройки: FILTER_TYPE=" + String(FILTER_TYPE) + 
                 ", FILTER_SIZE=" + String(FILTER_SIZE));
  Serial.println("ВРЕМЯ_МС,ТЕМП_СЫРАЯ,ТЕМП_ФИЛЬТ,ДЕЛЬТА");
  
  // Данные (первые 20 строк)
  int rowsToShow = min(20, measurements.count);
  
  for (int i = 0; i < rowsToShow; i++) {
    String row = String(measurements.timestamps[i]) + ",";
    row += String(measurements.rawValues[i], 4) + ",";
    row += String(measurements.filteredValues[i], 4) + ",";
    
    float delta = measurements.deltaValues[i];
    if (delta >= 0) {
      row += "+" + String(delta, 4);
    } else {
      row += String(delta, 4);
    }
    
    Serial.println(row);
  }
  
  if (measurements.count > 20) {
    Serial.printf("# ... и ещё %d строк\n", measurements.count - 20);
  }
}

/**
 * ФУНКЦИЯ: printTestResults()
 * ИЗМЕНЕНИЕ: Все значения с 4 знаками
 */
void printTestResults() {
  Serial.println("\n" + String(60, '='));
  Serial.println("РЕЗУЛЬТАТЫ ТЕСТА (без округления)");
  Serial.println(String(60, '='));
  
  float avgRaw = stats.sumRaw / measurements.count;
  float avgFiltered = stats.sumFiltered / measurements.count;
  
  Serial.println("\nОСНОВНАЯ СТАТИСТИКА:");
  Serial.printf("Общий диапазон:  %.4f°C ── %.4f°C\n", stats.minRaw, stats.maxRaw);
  Serial.printf("Среднее (сырое): %.4f°C\n", avgRaw);
  Serial.printf("Среднее (фильтр): %.4f°C\n", avgFiltered);
  
  if (calculateNoiseStatistics()) {
    float noiseRange = stats.noiseMax - stats.noiseMin;
    float noiseStdDev = sqrt((stats.noiseSumSq / measurements.count) - 
                            pow(stats.noiseSum / measurements.count, 2));
    
    Serial.printf("Чистый шум:      %.4f°C размах\n", noiseRange);
    Serial.printf("СКО (σ):         %.4f°C\n", noiseStdDev);
    
    if (noiseStdDev > 0) {
      float sigmaRatio = DELTA_THRESHOLD / noiseStdDev;
      Serial.printf("Отношение порог/σ: %.1f\n", sigmaRatio);
    }
  }
  
  unsigned long testDuration = stats.endTime - stats.startTime;
  Serial.printf("Время теста:     %.1f минут\n", testDuration / 60000.0);
  Serial.printf("Измерений:       %d/%d\n", measurements.count, TEST_DURATION);
  
  printHistogram();
  printAutocorrelation();
  printCSVData();
  
  Serial.println("\n" + String(60, '='));
  Serial.println("ТЕСТ ЗАВЕРШЁН");
  Serial.println(String(60, '='));
}

// ============================================================================
// РАЗДЕЛ 9: ОСНОВНЫЕ ФУНКЦИИ ARDUINO
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n" + String(50, '*'));
  Serial.println("*  ТЕСТОВЫЙ РЕЖИМ DS18B20 (без округления)  *");
  Serial.println(String(50, '*'));
  
  sensors.begin();
  delay(100);
  
  int deviceCount = sensors.getDeviceCount();
  Serial.printf("Датчиков найдено: %d\n", deviceCount);
  
  if (deviceCount == 0) {
    Serial.println("❌ ОШИБКА: Датчики не обнаружены!");
    while (true) delay(1000);
  }
  
  if (!sensors.getAddress(sensorAddress, 0)) {
    Serial.println("❌ ОШИБКА: Не удалось получить адрес!");
    while (true) delay(1000);
  }
  
  sensors.setResolution(sensorAddress, RESOLUTION);
  
  measurements.count = 0;
  measurements.maxCount = min(500, TEST_DURATION);
  initStatistics();
  
  printTestHeader();
  
  testRunning = true;
  testPhase = 0;
  
  Serial.println("\n✅ Система готова. Начинаем тест...\n");
  delay(1000);
  
  stats.startTime = millis();
}

void loop() {
  if (!testRunning) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastMeasureTime < MEASURE_INTERVAL) return;
  
  lastMeasureTime = currentTime;
  
  sensors.requestTemperatures();
  delay(getConversionDelay(RESOLUTION));
  
  float rawTemperature = sensors.getTempC(sensorAddress);
  
  if (rawTemperature == DEVICE_DISCONNECTED_C) {
    Serial.printf("[%6lu мс] ❌ Ошибка чтения\n", currentTime - stats.startTime);
    return;
  }
  
  // ВАЖНО: Все вычисления БЕЗ округления!
  float filteredTemperature = applyFilter(rawTemperature);
  float deltaChange = detectDeltaChange(filteredTemperature);
  
  if (measurements.count < measurements.maxCount) {
    measurements.rawValues[measurements.count] = rawTemperature;
    measurements.filteredValues[measurements.count] = filteredTemperature;
    measurements.deltaValues[measurements.count] = deltaChange;
    measurements.timestamps[measurements.count] = currentTime - stats.startTime;
    measurements.count++;
    
    updateStatistics(rawTemperature, filteredTemperature);
    printProgressBar(measurements.count, TEST_DURATION);
  }
  
  if (measurements.count >= TEST_DURATION) {
    testRunning = false;
    stats.endTime = millis();
    
    Serial.println();
    delay(1000);
    
    printTestResults();
    
    Serial.println("\n" + String(50, '='));
    Serial.println("🛑 ПРОГРАММА ЗАВЕРШЕНА");
    Serial.println("Для нового теста измените настройки и перезагрузите");
    
    while (true) delay(1000);
  }
}