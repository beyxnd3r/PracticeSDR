#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using IQ = std::complex<float>;
constexpr int SAMPLES_PER_BIT = 10;
constexpr int HEADER = 6;
constexpr double RATE = 1'000'000.0;
constexpr double FREQUENCY = 800e6;
constexpr float PI = 3.14159265358979323846f;

// Кадр BPSK: текст кодируется старшим битом вперёд.
std::vector<int16_t> makeFrame(const std::string& text, size_t mtu) {
    std::vector<int16_t> out(mtu * 2, 0);
    if (mtu <= HEADER) return out;
    out[0] = out[1] = out[10] = out[11] = -1;
    const size_t bits = std::min(text.size() * 8, (mtu - HEADER) / SAMPLES_PER_BIT);
    for (size_t k = 0; k < bits; ++k) {
        const unsigned char byte = static_cast<unsigned char>(text[k / 8]);
        const bool one = (byte >> (7 - k % 8)) & 1;
        const int16_t level = one ? -24000 : 24000; // 1500 << 4
        for (int j = 0; j < SAMPLES_PER_BIT; ++j)
            out[2 * (HEADER + k * SAMPLES_PER_BIT + j)] = level;
    }
    return out;
}

void putTimestamp(std::vector<int16_t>& frame, long long ns) {
    for (int j = 0; j < 8; ++j)
        frame[2 + j] = static_cast<int16_t>(((static_cast<uint64_t>(ns) >> (8 * j)) & 255) << 4);
}

// Прямоугольный согласованный фильтр: скользящее среднее по 10 отсчётам.
std::vector<IQ> averageSymbols(const std::vector<int16_t>& interleaved) {
    const size_t n = interleaved.size() / 2;
    std::vector<IQ> result(n);
    IQ sum(0, 0);
    for (size_t i = 0; i < n; ++i) {
        sum += IQ(interleaved[2 * i], interleaved[2 * i + 1]);
        if (i >= SAMPLES_PER_BIT)
            sum -= IQ(interleaved[2 * (i - SAMPLES_PER_BIT)],
                      interleaved[2 * (i - SAMPLES_PER_BIT) + 1]);
        result[i] = sum / static_cast<float>(SAMPLES_PER_BIT);
    }
    return result;
}

// Линейная интерполяция для дробного положения символьного строба.
IQ sampleAt(const std::vector<IQ>& data, float index) {
    const size_t a = static_cast<size_t>(std::floor(index));
    if (a + 1 >= data.size()) return data.back();
    const float fraction = index - static_cast<float>(a);
    return data[a] * (1.0f - fraction) + data[a + 1] * fraction;
}

struct TimingState {
    std::vector<IQ> decisions;
    float phase = 0;
    float averageError = 0;
    size_t tracked = 0;
};

TimingState recoverTiming(const std::vector<IQ>& samples) {
    TimingState result;
    if (samples.size() < 4 * SAMPLES_PER_BIT) return result;

    // Поиск начальной фазы по максимальной средней энергии символов.
    const size_t search = std::min(samples.size(), size_t(12000));
    float best = -1;
    int offset = 0;
    for (int shift = 0; shift < SAMPLES_PER_BIT; ++shift) {
        double power = 0;
        for (size_t i = shift + SAMPLES_PER_BIT; i < search; i += SAMPLES_PER_BIT)
            power += std::abs(samples[i]);
        if (power > best) { best = static_cast<float>(power); offset = shift; }
    }

    float maximum = 0;
    for (const IQ& z : samples) maximum = std::max(maximum, std::abs(z));
    const float gate = std::max(15.0f, maximum * 0.20f);
    float phase = static_cast<float>(offset + SAMPLES_PER_BIT);
    float correction = 0;
    double accumulated = 0;
    while (phase + 2 < samples.size()) {
        const IQ now = sampleAt(samples, phase);
        const IQ before = sampleAt(samples, phase - SAMPLES_PER_BIT);
        const IQ halfway = sampleAt(samples, phase - SAMPLES_PER_BIT / 2.0f);
        result.decisions.push_back(now);

        float step = 0;
        if (std::abs(now) > gate && std::abs(before) > gate) {
            const float scale = 0.5f * (std::norm(now) + std::norm(before)) + 1e-5f;
            const float error = std::clamp(std::real((now - before) * std::conj(halfway)) / scale,
                                           -1.0f, 1.0f);
            correction = std::clamp(correction + 0.015f * error, -0.35f, 0.35f);
            step = 0.25f * error + correction;
            accumulated += std::abs(error);
            ++result.tracked;
        }
        phase += std::clamp(SAMPLES_PER_BIT - step, 9.0f, 11.0f);
    }
    result.phase = std::fmod(phase, static_cast<float>(SAMPLES_PER_BIT));
    result.averageError = result.tracked ? accumulated / result.tracked : 0;
    return result;
}

void savePcm(const std::vector<int16_t>& iq, const char* name) {
    if (FILE* file = std::fopen(name, "wb")) {
        std::fwrite(iq.data(), sizeof(int16_t), iq.size(), file);
        std::fclose(file);
    } else std::perror(name);
}

// Модель приёмного радиоканала: CFO, шум и неидеальный коэффициент усиления.
std::vector<int16_t> simulateChannel(const std::vector<int16_t>& tx, size_t frames) {
    const size_t frameSize = tx.size() / 2;
    std::vector<int16_t> rx(frames * tx.size());
    std::mt19937 engine(9);
    std::normal_distribution<float> noise(0.0f, 13.0f);
    const float cfo = 320.0f;
    for (size_t k = 0; k < frames; ++k) {
        for (size_t i = 0; i < frameSize; ++i) {
            const size_t t = k * frameSize + i;
            const float angle = 2 * PI * cfo * static_cast<float>(t / RATE);
            const IQ source(tx[2 * i], tx[2 * i + 1]);
            const IQ rotated = source * std::polar(0.018f, angle);
            const IQ value = rotated + IQ(noise(engine), noise(engine));
            rx[2 * t] = static_cast<int16_t>(std::clamp(std::round(value.real()), -2047.0f, 2047.0f));
            rx[2 * t + 1] = static_cast<int16_t>(std::clamp(std::round(value.imag()), -2047.0f, 2047.0f));
        }
    }
    return rx;
}

std::vector<int16_t> capturePluto(const std::string& uri) {
    SoapySDRKwargs config = {};
    SoapySDRKwargs_set(&config, "driver", "plutosdr");
    SoapySDRKwargs_set(&config, "uri", uri.c_str());
    SoapySDRKwargs_set(&config, "direct", "1");
    SoapySDRKwargs_set(&config, "timestamp_every", "1920");
    SoapySDRKwargs_set(&config, "loopback", "0");
    SoapySDRDevice* device = SoapySDRDevice_make(&config);
    SoapySDRKwargs_clear(&config);
    if (!device) {
        std::cerr << "PlutoSDR недоступен: " << SoapySDRDevice_lastError() << '\n';
        return {};
    }
    for (int direction : {SOAPY_SDR_RX, SOAPY_SDR_TX}) {
        SoapySDRDevice_setSampleRate(device, direction, 0, RATE);
        SoapySDRDevice_setFrequency(device, direction, 0, FREQUENCY, nullptr);
        SoapySDRDevice_setGain(device, direction, 0, direction == SOAPY_SDR_RX ? 10.0 : -40.0);
    }
    size_t channels[] = {0};
    auto* rx = SoapySDRDevice_setupStream(device, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, 1, nullptr);
    auto* tx = SoapySDRDevice_setupStream(device, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, 1, nullptr);
    if (!rx || !tx) {
        std::cerr << "Ошибка настройки потоков\n";
        if (rx) SoapySDRDevice_closeStream(device, rx);
        if (tx) SoapySDRDevice_closeStream(device, tx);
        SoapySDRDevice_unmake(device);
        return {};
    }
    SoapySDRDevice_activateStream(device, rx, 0, 0, 0);
    SoapySDRDevice_activateStream(device, tx, 0, 0, 0);
    const size_t rxMtu = SoapySDRDevice_getStreamMTU(device, rx);
    const size_t txMtu = SoapySDRDevice_getStreamMTU(device, tx);
    std::vector<int16_t> buffer(2 * rxMtu);
    auto frame = makeFrame("PracticeSDR!", txMtu);
    std::vector<int16_t> recorded;
    recorded.reserve(300 * 2 * rxMtu);
    for (int round = 0; round < 300; ++round) {
        void* input[] = {buffer.data()};
        int flags = 0;
        long long timestamp = 0;
        const int received = SoapySDRDevice_readStream(device, rx, input, rxMtu, &flags, &timestamp, 200000);
        if (received <= 0) continue;
        recorded.insert(recorded.end(), buffer.begin(), buffer.begin() + 2 * received);
        if (round >= 20 && round % 10 == 0 && (flags & SOAPY_SDR_HAS_TIME)) {
            const long long sendAt = timestamp + 4'000'000LL;
            putTimestamp(frame, sendAt);
            const void* output[] = {frame.data()};
            int txFlags = SOAPY_SDR_HAS_TIME;
            const int written = SoapySDRDevice_writeStream(device, tx, output, txMtu,
                                                           &txFlags, sendAt, 200000);
            if (written < 0) std::cerr << "Ошибка TX: " << written << '\n';
        }
    }
    SoapySDRDevice_deactivateStream(device, rx, 0, 0);
    SoapySDRDevice_deactivateStream(device, tx, 0, 0);
    SoapySDRDevice_closeStream(device, rx);
    SoapySDRDevice_closeStream(device, tx);
    SoapySDRDevice_unmake(device);
    return recorded;
}

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "offline";
    std::vector<int16_t> iq;
    if (mode == "offline") {
        iq = simulateChannel(makeFrame("PracticeSDR!", 1920), 300);
        std::cout << "Программная симуляция, PlutoSDR не используется\n";
    } else if (mode == "pluto") {
        iq = capturePluto(argc > 2 ? argv[2] : "usb:");
    } else {
        std::cerr << "Использование: " << argv[0] << " offline | pluto [uri]\n";
        return 1;
    }
    if (iq.empty()) return 1;
    savePcm(iq, "rxdata.pcm");
    const TimingState sync = recoverTiming(averageSymbols(iq));
    std::cout << "Отсчётов: " << iq.size() / 2
              << ", символов: " << sync.decisions.size()
              << ", фаза: " << sync.phase
              << ", средняя ошибка: " << sync.averageError
              << ", активных решений: " << sync.tracked << '\n';
    return 0;
}
