# FT8 Decoder Roadmap для Portapack Mayhem

## Текущий статус (Phase 0) ✅

**Что сделано:**
- ✅ Создан `ft8_rx` external app
- ✅ Базовый UI с контролами (LNA, VGA, RFAmp, RSSI)
- ✅ Правильная архитектура (RxRadioState, SettingsManager, MessageHandlerRegistration)
- ✅ Адрес: 0xADF00000
- ✅ Частота: 7.074 MHz, полоса: 3 kHz
- ✅ Компилируется и собирается (1800 байт)
- ✅ Использует стандартный PCAP baseband (capture mode)

**Ограничения:**
- ❌ Нет FT8 decoder на baseband (M4)
- ❌ Нет декодирования сигналов
- ❌ Нет waterfall display
- ❌ Нет списка декодированных станций

---

## Phase 1: Baseband Audio Capture & Basic UI

**Цель:** Получить аудио с IQ → Audio pipeline и визуализировать

### 1.1 Baseband Audio Processing

**Создать:** `/firmware/baseband/proc_ft8_rx.cpp`

```cpp
class FT8RxProcessor : public BasebandProcessor {
public:
    void execute(const buffer_c8_t& buffer) override {
        // 1. Decimation chain
        // 2457600 Hz → 307200 Hz → 38400 Hz → 12000 Hz
        const auto decim_0_out = decim_0.execute(buffer, dst_buffer);
        const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);
        const auto channel_out = channel_filter.execute(decim_1_out, dst_buffer);

        // 2. FM demodulation (or USB for SSB)
        auto audio = demod.execute(channel_out, audio_buffer);

        // 3. Output audio для мониторинга
        audio_output.write(audio);

        // 4. TODO Phase 2: FFT analysis
    }

private:
    dsp::decimate::FIRC8xR16x24FS4Decim8 decim_0{};
    dsp::decimate::FIRC16xR16x32Decim8 decim_1{};
    dsp::decimate::FIRAndDecimateComplex channel_filter{};
    dsp::demodulate::FM demod{};  // Или USB для SSB

    BasebandThread baseband_thread{12000, this, Direction::Receive};
    RSSIThread rssi_thread{};
};
```

**Обновить:** `main.cpp` для загрузки нового baseband
```cpp
/*.m4_app_tag = */ {'P', 'F', 'T', '8'},  // Новый тег
```

### 1.2 Enhanced UI with Waterfall

**Добавить в `ui_ft8_rx.hpp`:**

```cpp
class FT8RxView : public View {
public:
    // ... existing code ...

private:
    // Add waterfall
    spectrum::WaterfallWidget waterfall{};

    // Add decoded messages console
    Console console{
        {0, 200, 240, 80}
    };

    // Message handler для спектра
    MessageHandlerRegistration message_handler_spectrum{
        Message::ID::ChannelSpectrum,
        [this](const Message* const p) {
            auto spectrum = static_cast<const ChannelSpectrumMessage*>(p);
            waterfall.on_channel_spectrum(spectrum->spectrum);
        }
    };
};
```

**Задачи:**
- [ ] Создать proc_ft8_rx.cpp
- [ ] Настроить USB demodulation (3 kHz)
- [ ] Добавить waterfall widget
- [ ] Тест аудио выхода
- [ ] Создать PFT8.bin baseband image

**Результат Phase 1:**
- ✅ Слышим аудио FT8 сигналов
- ✅ Видим waterfall
- ✅ Можем визуально увидеть FT8 сигналы на частоте

---

## Phase 2: FFT Analysis & Symbol Detection

**Цель:** Детектировать FT8 сигналы через FFT

### 2.1 FFT-based Signal Detection

FT8 использует 8-FSK модуляцию:
- 8 тонов разнесены на 6.25 Hz
- Symbol rate: 6.25 symbols/sec
- Symbol duration: 0.16 sec (160 ms)
- Total bandwidth: ~50 Hz на сигнал

**Добавить в proc_ft8_rx.cpp:**

```cpp
class FT8RxProcessor : public BasebandProcessor {
    void execute(const buffer_c8_t& buffer) override {
        // ... existing decimation & demod ...

        // FFT analysis каждые 160ms
        for (size_t i = 0; i < audio.count; i++) {
            fft_buffer[fft_index++] = audio.p[i];

            if (fft_index >= fft_size) {
                perform_fft();
                detect_ft8_signals();
                fft_index = 0;
            }
        }
    }

    void perform_fft() {
        // Используем arm_rfft_fast_f32 из CMSIS-DSP
        arm_rfft_fast_f32(&rfft_instance, fft_buffer, fft_output, 0);
        arm_cmplx_mag_f32(fft_output, fft_magnitudes, fft_size / 2);
    }

    void detect_ft8_signals() {
        // Ищем пики в диапазоне 0-3000 Hz
        // Минимум 8 тонов на расстоянии ~6.25 Hz

        for (int bin = 0; bin < max_bin; bin++) {
            if (fft_magnitudes[bin] > threshold) {
                // Потенциальный FT8 сигнал
                Signal signal;
                signal.frequency = bin_to_hz(bin);
                signal.magnitude = fft_magnitudes[bin];
                signal.time_offset = current_time_slot;

                signals.push_back(signal);
            }
        }

        // Отправка в M0 для отображения
        if (!signals.empty()) {
            FT8SignalMessage message{signals};
            shared_memory.application_queue.push(message);
        }
    }

private:
    static constexpr size_t fft_size = 2048;
    float fft_buffer[fft_size];
    float fft_output[fft_size];
    float fft_magnitudes[fft_size / 2];

    arm_rfft_fast_instance_f32 rfft_instance;

    std::vector<Signal> signals;
};
```

### 2.2 UI Updates

**Добавить в ui_ft8_rx.cpp:**

```cpp
void FT8RxView::on_ft8_signals(const FT8SignalMessage* message) {
    // Display detected signals
    for (const auto& signal : message->signals) {
        char buf[64];
        sprintf(buf, "Signal @ %4d Hz: %.1f dB",
                signal.frequency, signal.magnitude);
        console.writeln(buf);
    }
}

// Message handler
MessageHandlerRegistration message_handler_ft8_signals{
    Message::ID::FT8Signal,
    [this](const Message* const p) {
        on_ft8_signals(static_cast<const FT8SignalMessage*>(p));
    }
};
```

**Задачи:**
- [ ] Реализовать FFT на baseband (CMSIS-DSP)
- [ ] Детектировать пики (8-FSK тоны)
- [ ] Отправлять координаты сигналов в M0
- [ ] Отображать детекции в консоли
- [ ] Визуализировать на waterfall

**Результат Phase 2:**
- ✅ Видим когда присутствуют FT8 сигналы
- ✅ Знаем частоту каждого сигнала
- ✅ Можем оценить SNR

---

## Phase 3: FT8 Protocol Decoder (Partial)

**Цель:** Декодировать символы и синхронизацию

### 3.1 FT8 Symbol Extraction

FT8 message structure:
- 79 symbols total
- First 7 symbols: Costas array (sync)
- Symbol duration: 0.16 sec
- Total message: 12.64 sec

**Costas array (sync pattern):**
```
[3, 1, 4, 0, 6, 5, 2]
```

**Добавить symbol extraction:**

```cpp
class FT8Decoder {
public:
    struct Symbol {
        uint8_t tone;      // 0-7
        float magnitude;
        float snr;
    };

    bool decode_symbols(const float* fft_magnitudes, int base_freq) {
        for (int sym = 0; sym < 79; sym++) {
            int best_tone = -1;
            float best_mag = 0;

            // Найти максимум среди 8 тонов
            for (int tone = 0; tone < 8; tone++) {
                int freq = base_freq + tone * 625;  // 6.25 Hz spacing
                int bin = hz_to_bin(freq);

                if (fft_magnitudes[bin] > best_mag) {
                    best_mag = fft_magnitudes[bin];
                    best_tone = tone;
                }
            }

            symbols[sym] = {best_tone, best_mag, calculate_snr(best_mag)};
        }

        // Проверка Costas sync
        if (check_costas_sync()) {
            return decode_message();
        }

        return false;
    }

    bool check_costas_sync() {
        const uint8_t costas[] = {3, 1, 4, 0, 6, 5, 2};

        // Sync в начале (symbols 0-6)
        for (int i = 0; i < 7; i++) {
            if (symbols[i].tone != costas[i])
                return false;
        }

        // Sync в середине (symbols 36-42)
        for (int i = 0; i < 7; i++) {
            if (symbols[36 + i].tone != costas[i])
                return false;
        }

        // Sync в конце (symbols 72-78)
        for (int i = 0; i < 7; i++) {
            if (symbols[72 + i].tone != costas[i])
                return false;
        }

        return true;
    }

private:
    Symbol symbols[79];
};
```

**Задачи:**
- [ ] Извлечение 79 символов из FFT
- [ ] Детекция Costas sync pattern
- [ ] Определение time offset (0-15 sec)
- [ ] Расчёт SNR

**Результат Phase 3:**
- ✅ Синхронизация с FT8 сигналами
- ✅ Извлечение символов (79 tones)
- ✅ Детекция начала сообщения

---

## Phase 4: FT8 Message Decoding (Full)

**Цель:** Полное декодирование FT8 протокола

### 4.1 LDPC Error Correction

FT8 использует LDPC (Low-Density Parity-Check) для исправления ошибок:
- 174 информационных бита
- 91 информационный + 83 parity bits
- Code rate: 91/174

**Это самая сложная часть!** Требует:
- LDPC decoder алгоритм
- Belief propagation
- Итеративное декодирование

**Примерный код (упрощённо):**

```cpp
class LDPCDecoder {
public:
    bool decode(const uint8_t* symbols_79, uint8_t* message_out) {
        // 1. Gray code mapping (symbols → bits)
        uint8_t bits[174];
        symbols_to_bits(symbols_79, bits);

        // 2. LDPC decoding (итеративный)
        float llr[174];  // Log-likelihood ratios
        init_llr(bits, llr);

        for (int iter = 0; iter < max_iterations; iter++) {
            // Belief propagation algorithm
            check_node_update(llr);
            variable_node_update(llr);

            if (check_parity(llr)) {
                extract_message(llr, message_out);
                return true;
            }
        }

        return false;  // Decoding failed
    }

private:
    // H-matrix для FT8 (174x91)
    static const uint16_t parity_check_matrix[174][91];

    void symbols_to_bits(const uint8_t* symbols, uint8_t* bits);
    void check_node_update(float* llr);
    void variable_node_update(float* llr);
    bool check_parity(const float* llr);
};
```

### 4.2 Message Unpacking

После LDPC декодирования получаем 91 бит. Нужно распаковать:

```cpp
struct FT8Message {
    enum Type {
        STANDARD_MSG = 0,
        EU_VHF_CONTEST = 1,
        RTTY_ROUNDUP = 2,
        // ... другие типы
    };

    Type type;
    char call_from[16];
    char call_to[16];
    char grid_locator[8];
    int snr_report;

    static FT8Message unpack(const uint8_t* bits_91) {
        FT8Message msg;

        // i3 (3 bits) - message type
        msg.type = static_cast<Type>(extract_bits(bits_91, 0, 3));

        if (msg.type == STANDARD_MSG) {
            // n3 (1 bit) - ...
            // call_from (28 bits) - callsign encoding
            unpack_callsign(extract_bits(bits_91, 4, 28), msg.call_from);

            // call_to (28 bits)
            unpack_callsign(extract_bits(bits_91, 32, 28), msg.call_to);

            // grid/report (15 bits)
            unpack_grid_or_report(extract_bits(bits_91, 60, 15),
                                  msg.grid_locator, msg.snr_report);
        }

        return msg;
    }
};
```

**Задачи:**
- [ ] Реализовать LDPC decoder (сложно!)
- [ ] Gray code demapping
- [ ] Message unpacking (callsigns, grid, SNR)
- [ ] CRC проверка
- [ ] Hash table для callsign lookup

**Результат Phase 4:**
- ✅ Полное декодирование FT8 сообщений
- ✅ Извлечение: callsign, grid locator, SNR report
- ✅ Коррекция ошибок

---

## Phase 5: Advanced UI & Features

**Цель:** Профессиональный UI и дополнительные функции

### 5.1 Station List with Auto-logging

```cpp
class FT8StationList : public Widget {
public:
    struct Station {
        char callsign[16];
        char grid[8];
        int snr;
        uint32_t frequency;
        time_t last_seen;
        bool is_new;  // Новая станция в этой сессии
    };

    void add_station(const FT8Message& msg, uint32_t freq);
    void paint(Painter& painter) override;

private:
    std::vector<Station> stations_;

    // Сортировка по SNR или времени
    enum SortMode { BY_SNR, BY_TIME, BY_CALL } sort_mode_;
};
```

### 5.2 Band Activity Waterfall with Annotations

```cpp
class FT8Waterfall : public spectrum::WaterfallWidget {
public:
    void add_decode(const FT8Message& msg, uint32_t freq, time_t time) {
        Annotation ann;
        ann.frequency = freq;
        ann.time = time;
        ann.text = msg.call_from;
        ann.color = ui::Color::green();
        annotations_.push_back(ann);
    }

    void paint(Painter& painter) override {
        // Draw waterfall
        WaterfallWidget::paint(painter);

        // Overlay annotations
        for (const auto& ann : annotations_) {
            draw_annotation(painter, ann);
        }
    }

private:
    std::vector<Annotation> annotations_;
};
```

### 5.3 Auto-logging to SD Card

```cpp
class FT8Logger : public Logger {
public:
    FT8Logger() : Logger("FT8", "ft8_") {}

    void log_decode(const FT8Message& msg, uint32_t freq, float snr) {
        char timestamp[32];
        rtc_time::format_time_iso(timestamp);

        log_str(format_string(
            "%s,%d,%.1f,%s,%s,%s,%d",
            timestamp,
            freq,
            snr,
            msg.call_from,
            msg.call_to,
            msg.grid_locator,
            msg.snr_report
        ));
    }
};
```

**Задачи:**
- [ ] Список станций с фильтрацией
- [ ] Waterfall с аннотациями
- [ ] Auto-logging в CSV
- [ ] Time sync check (важно для FT8!)
- [ ] Band plan selector (40m, 20m, 15m, ...)

**Результат Phase 5:**
- ✅ Профессиональный UI
- ✅ Список декодированных станций
- ✅ Автоматический лог в CSV
- ✅ Визуализация активности

---

## Phase 6: TX Support (Optional, Advanced)

**Цель:** Добавить передачу FT8

### 6.1 Message Encoding

```cpp
class FT8Encoder {
public:
    void encode(const char* call_from, const char* call_to,
                const char* grid, uint8_t* symbols_out) {
        uint8_t bits[91];

        // Pack message
        pack_callsign(call_from, bits, 0);
        pack_callsign(call_to, bits, 28);
        pack_grid(grid, bits, 56);

        // LDPC encoding
        uint8_t encoded[174];
        ldpc_encode(bits, encoded);

        // Add Costas sync
        add_costas_sync(encoded, symbols_out);
    }
};
```

### 6.2 Baseband TX

```cpp
class FT8TxProcessor : public BasebandProcessor {
    void execute(const buffer_c8_t& buffer) override {
        for (size_t i = 0; i < buffer.count; i++) {
            // Generate 8-FSK tone
            float phase_inc = tone_to_phase_inc(current_tone);
            tx_phase += phase_inc;

            // I/Q generation
            float i_sample = cos(tx_phase);
            float q_sample = sin(tx_phase);

            buffer.p[i] = {
                static_cast<int8_t>(i_sample * 127),
                static_cast<int8_t>(q_sample * 127)
            };

            // Next symbol каждые 160ms
            if (++sample_count >= samples_per_symbol) {
                current_tone = symbols[++symbol_index];
                sample_count = 0;
            }
        }
    }

private:
    uint8_t symbols[79];
    int symbol_index = 0;
    int sample_count = 0;
    float tx_phase = 0;
};
```

**Задачи:**
- [ ] Message encoding & LDPC
- [ ] 8-FSK modulation
- [ ] Time sync (начало на 0, 15, 30, 45 sec)
- [ ] UI для TX
- [ ] Sequencer (RX → TX switching)

**Результат Phase 6:**
- ✅ Полнодуплексный FT8 трансивер
- ✅ Автоответы (CQ, reports)
- ✅ QSO logging

---

## Technical Challenges & Solutions

### Challenge 1: LDPC Decoder Complexity

**Проблема:** LDPC decoder требует много CPU и памяти

**Решения:**
1. **Option A:** Реализовать упрощённый hard-decision decoder
   - Проще, но хуже коррекция ошибок
   - Подходит для сильных сигналов (SNR > 0 dB)

2. **Option B:** Использовать min-sum algorithm
   - Проще чем belief propagation
   - Хорошая производительность

3. **Option C:** Offload на M0
   - Baseband только FFT и symbol extraction
   - M0 делает LDPC (медленнее, но возможно)

### Challenge 2: Memory Constraints

**M4 Memory:**
- 128 KB RAM total
- Shared с другими процессорами
- FFT buffers занимают много

**Решения:**
- Streaming FFT (не хранить все 12.64 sec)
- Декодировать по мере поступления символов
- Переиспользовать буферы

### Challenge 3: Timing & Synchronization

FT8 требует точной синхронизации:
- Message начинается каждые 15 секунд (0, 15, 30, 45)
- Нужен точный RTC

**Решения:**
- Использовать `rtc_time::signal_tick_second`
- GPS time sync (если доступен GPS)
- Грубая sync по детекции Costas pattern

### Challenge 4: Float Processing on M4

**Проблема:** M4 имеет FPU, но float операции медленнее int

**Решения:**
- Использовать CMSIS-DSP оптимизации
- Fixed-point где возможно
- SIMD инструкции (NEON не доступен на M4)

---

## Resource Requirements

### Code Size Estimates

| Phase | Application (M0) | Baseband (M4) | Total |
|-------|------------------|---------------|-------|
| Phase 0 (current) | 1.8 KB | 0 KB | 1.8 KB |
| Phase 1 | +2 KB | +4 KB | ~8 KB |
| Phase 2 | +3 KB | +6 KB | ~17 KB |
| Phase 3 | +2 KB | +8 KB | ~27 KB |
| Phase 4 | +4 KB | +12 KB | ~43 KB |
| Phase 5 | +5 KB | +2 KB | ~50 KB |
| Phase 6 (TX) | +6 KB | +8 KB | ~64 KB |

**Limit:** 32 KB per external app (без baseband)

### RAM Requirements

| Component | Size |
|-----------|------|
| FFT buffers (2048) | ~16 KB |
| Symbol buffers | ~2 KB |
| LDPC decoder state | ~8 KB |
| Message buffers | ~2 KB |
| **Total** | **~28 KB** |

**M4 RAM:** 128 KB (достаточно, но нужна оптимизация)

---

## Testing Strategy

### Phase 1 Testing
- [ ] Подать тестовый FT8 сигнал через RF
- [ ] Проверить аудио выход
- [ ] Визуально увидеть сигналы на waterfall

### Phase 2 Testing
- [ ] Записать IQ данные с реального FT8
- [ ] Replay через hackrf_transfer
- [ ] Проверить детекцию всех сигналов

### Phase 3 Testing
- [ ] Генерировать тестовые FT8 сообщения (WSJT-X)
- [ ] Проверить синхронизацию Costas
- [ ] Измерить точность symbol extraction

### Phase 4 Testing
- [ ] Unit tests для LDPC decoder
- [ ] Проверить на известных сообщениях
- [ ] Тест с различными SNR (-10 dB ... +10 dB)

### Phase 5 Testing
- [ ] Long-term stability test (24 hours)
- [ ] Проверить логирование
- [ ] Memory leak detection

---

## Development Timeline

| Phase | Estimated Time | Dependencies |
|-------|----------------|--------------|
| Phase 0 | ✅ Complete | - |
| Phase 1 | 1-2 weeks | DSP knowledge, baseband API |
| Phase 2 | 2-3 weeks | FFT, signal processing |
| Phase 3 | 3-4 weeks | FT8 protocol understanding |
| Phase 4 | 6-8 weeks | LDPC, error correction theory |
| Phase 5 | 2-3 weeks | UI design |
| Phase 6 | 4-6 weeks | TX architecture, timing |
| **Total** | **18-26 weeks** (~4-6 months) | |

---

## References & Resources

### FT8 Protocol
- **WSJT-X Source Code:** https://sourceforge.net/projects/wsjt/
- **FT8 Protocol Spec:** Joe Taylor K1JT papers
- **ft8_lib:** https://github.com/kgoba/ft8_lib (reference decoder)

### DSP & Algorithms
- **CMSIS-DSP Library:** ARM optimized DSP functions
- **LDPC Codes:** "Modern Coding Theory" by Tom Richardson
- **Belief Propagation:** Tutorial papers

### Portapack Development
- **Mayhem Wiki:** https://github.com/portapack-mayhem/mayhem-firmware/wiki
- **External Apps Guide:** firmware/application/external/README.md
- **Discord Community:** Real-time help

### Tools
- **WSJT-X:** FT8 reference implementation & testing
- **hackrf_transfer:** IQ capture/replay for testing
- **GNU Radio:** Signal processing prototyping

---

## Next Immediate Steps

1. **Изучить существующие baseband процессоры:**
   - Прочитать `proc_ais.cpp` (похожий FSK)
   - Прочитать `proc_pocsag2.cpp` (FFT + sync detection)
   - Понять DSP pipeline

2. **Создать Phase 1 baseband:**
   - Скопировать структуру из `proc_ais.cpp`
   - Настроить USB demodulation (3 kHz)
   - Добавить аудио выход
   - Собрать и протестировать

3. **Обновить UI с waterfall:**
   - Добавить `WaterfallWidget`
   - Подключить spectrum messages
   - Визуально подтвердить работу

4. **Написать тесты:**
   - Записать IQ с реального FT8 (hackrf_transfer)
   - Создать test harness для replay
   - Автоматизировать проверки

---

## Success Criteria

### Phase 1 Success:
- ✅ Слышим FT8 аудио
- ✅ Видим сигналы на waterfall

### Phase 2 Success:
- ✅ Детектируем все FT8 сигналы в полосе
- ✅ Точность определения частоты: ±10 Hz

### Phase 3 Success:
- ✅ Синхронизация с 95%+ сигналами
- ✅ Извлечение всех 79 символов

### Phase 4 Success:
- ✅ Декодирование 80%+ сообщений при SNR > -5 dB
- ✅ 50%+ при SNR > -10 dB (как WSJT-X)

### Ultimate Goal:
- ✅ **Portable FT8 decoder на Portapack H2**
- ✅ Сравнимая с WSJT-X чувствительность
- ✅ Real-time декодирование
- ✅ Автологирование QSO
- ✅ Опционально: TX support

---

## Contribution Guidelines

### Code Style
- Следовать существующим конвенциям Mayhem
- Комментарии на английском
- Документировать алгоритмы

### Testing
- Unit tests где возможно
- Integration tests с реальными сигналами
- Performance benchmarks

### Documentation
- Обновлять этот roadmap
- Комментировать сложные алгоритмы
- Примеры использования

### Community
- Создать issue в Mayhem repo
- Делиться прогрессом в Discord
- Запросить feedback от сообщества

---

**Автор:** Claude (AI Assistant)
**Дата:** 2025-11-02
**Статус:** Phase 0 Complete, Ready for Phase 1
**Лицензия:** GPL (как Mayhem firmware)
