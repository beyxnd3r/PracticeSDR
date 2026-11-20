// Примеры запуска для двух направлений радиоканала:
// ./main tx [uri] -> TX 821 МГц, RX 766 МГц
// ./main rx [uri] -> TX 766 МГц, RX 821 МГц

#include <SoapySDR/Device.h> // создание дескриптора SDR устройства
#include <SoapySDR/Formats.h>  // Типы данных, используемых для записи сэмплов
#include <complex>
#include <stdint.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include <math.h> 
#include <signal.h> 

// Формирование цифрового потока передатчика
#define SYMBOL_SAMPLES 10 // отсчётов на один символ
#define SYMBOL_AMPLITUDE 2000  // уровень созвездия в формате ЦАП
#define FRAME_PREFIX_SAMPLES 6 // служебные комплексные отсчёты для заголовка

typedef enum { TX_BPSK, TX_QPSK } constellation_t;

// Прямоугольная импульсная характеристика для формирования символов
static const int16_t pulse_kernel[SYMBOL_SAMPLES] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

static void fill_random_bits(uint8_t *random_bits, size_t bit_count)
{
    for (size_t i = 0; i < bit_count; ++i) {
        random_bits[i] = static_cast<uint8_t>(rand() % 2);
    }
}

static void encode_bpsk(const uint8_t *random_bits, int16_t *mapped_buffer, size_t symbol_count)
{
    for (size_t i = 0; i < symbol_count; i++)
    {
        mapped_buffer[2 * i] = random_bits[i] ? -SYMBOL_AMPLITUDE : SYMBOL_AMPLITUDE;
        mapped_buffer[2 * i + 1] = 0;
    }
}

static void encode_qpsk(const uint8_t *random_bits, int16_t *mapped_buffer, size_t symbol_count)
{
    for (size_t i = 0; i < symbol_count; i++)
    {
        mapped_buffer[2 * i] = random_bits[2 * i] ? -SYMBOL_AMPLITUDE : SYMBOL_AMPLITUDE;
        mapped_buffer[2 * i + 1] = random_bits[2 * i + 1] ? -SYMBOL_AMPLITUDE : SYMBOL_AMPLITUDE;
    }
}

static void insert_zero_samples(const int16_t *input_symbols, size_t symbol_count,
                           int16_t *output_samples, size_t samples_per_symbol)
{
    for (size_t i = 0; i < symbol_count; i++)
    {
        const size_t base = 2 * i * samples_per_symbol;
        output_samples[base] = input_symbols[2 * i];
        output_samples[base + 1] = input_symbols[2 * i + 1];
        for (size_t j = 1; j < samples_per_symbol; ++j) {
            output_samples[base + 2 * j] = 0;
            output_samples[base + 2 * j + 1] = 0;
        }
    }
}

static void apply_pulse_filter(const int16_t *upsampled_data, int16_t *filtered_data,
                        size_t total_count)
{
    for (size_t i = 0; i < total_count; i++)
    {
        int32_t i_accumulator = 0, q_accumulator = 0;
        const size_t tap_count = (i + 1 < SYMBOL_SAMPLES) ? i + 1 : SYMBOL_SAMPLES;
        for (size_t j = 0; j < tap_count; ++j) {
            i_accumulator += upsampled_data[2 * (i - j)] * pulse_kernel[j];
            q_accumulator += upsampled_data[2 * (i - j) + 1] * pulse_kernel[j];
        }
        filtered_data[2 * i] = (int16_t)i_accumulator;
        filtered_data[2 * i + 1] = (int16_t)q_accumulator;
    }
}


static void fill_transmit_frame(int16_t *tx_data, size_t tx_block_size, constellation_t constellation)
{
    size_t symbol_count = (tx_block_size - FRAME_PREFIX_SAMPLES) / SYMBOL_SAMPLES;
    size_t payload_size = symbol_count * SYMBOL_SAMPLES;
    size_t bits_per_symbol = (constellation == TX_QPSK) ? 2 : 1;  // число бит, кодируемых символом

    uint8_t *random_bits = (uint8_t *)malloc(symbol_count * bits_per_symbol);
    int16_t *symbol_data = (int16_t *)malloc(symbol_count * 2 * sizeof(int16_t));
    int16_t *zero_padded = (int16_t *)malloc(payload_size * 2 * sizeof(int16_t));
    int16_t *filtered_symbols  = (int16_t *)malloc(payload_size * 2 * sizeof(int16_t));

    fill_random_bits(random_bits, symbol_count * bits_per_symbol);
    if (constellation == TX_QPSK) encode_qpsk(random_bits, symbol_data, symbol_count);
    else encode_bpsk(random_bits, symbol_data, symbol_count);

    insert_zero_samples(symbol_data, symbol_count, zero_padded, SYMBOL_SAMPLES);
    apply_pulse_filter(zero_padded, filtered_symbols, payload_size);

    int16_t *payload_start = tx_data + 2 * FRAME_PREFIX_SAMPLES;
    memset(payload_start, 0, 2 * (tx_block_size - FRAME_PREFIX_SAMPLES) * sizeof(int16_t));
    // Переносим в буфер пары I/Q, сохраняя выравнивание CS16.
    for (size_t i = 0; i < payload_size; ++i) {
        payload_start[2 * i] = filtered_symbols[2 * i] * 16;
        payload_start[2 * i + 1] = filtered_symbols[2 * i + 1] * 16;
    }

    free(random_bits); free(symbol_data); free(zero_padded); free(filtered_symbols);
}

#define RADIO_SAMPLE_RATE 1e6
#define RECEIVER_GAIN 30.0
#define TRANSMITTER_GAIN -40.0
#define STATUS_PERIOD 250 // период вывода статистики
#define NOISE_THRESHOLD 20 // порог для отметки слабого входного сигнала
#define SELECTED_MODULATION TX_QPSK // схема модуляции по умолчанию

static const size_t RX_WARMUP_BUFFERS = 128;
static const long long TX_ADVANCE_NS = 4000000LL; // передача через 4 мс

double upper_frequency = 821e6;
double lower_frequency = 766e6;

static volatile sig_atomic_t work_active = 1;
static void stop_on_interrupt(int interrupt_code) {
  (void)interrupt_code;
  work_active = 0;
  signal(SIGINT, SIG_DFL);
}

SoapySDRDevice *create_radio(const char *radio_uri) {
  SoapySDRKwargs device_settings = {};
  SoapySDRKwargs_set(&device_settings, "driver","plutosdr");
  SoapySDRKwargs_set(&device_settings, "uri", radio_uri);
  SoapySDRKwargs_set(&device_settings, "direct", "1");  //
  SoapySDRKwargs_set(&device_settings, "timestamp_every","1920");  // метки времени через указанный интервал
  SoapySDRKwargs_set(&device_settings, "loopback", "0");  // отключение программного loopback
  SoapySDRDevice *radio = SoapySDRDevice_make(&device_settings);  // создание дескриптора SDR
  SoapySDRKwargs_clear(&device_settings);
  return radio;
}


int start_radio(const char *radio_uri, bool tx_mode) {
  SoapySDRDevice *radio = create_radio(radio_uri);
  if (radio == NULL) {
    printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
    return 1;
  }

  double receive_frequency = upper_frequency;
  double transmit_frequency = lower_frequency;
  const char *mode_label = "rx";

  if (tx_mode) {
    receive_frequency = lower_frequency;
    transmit_frequency = upper_frequency;
    mode_label = "tx";
  }
  printf("Режим: %s, устройство: %s\n", mode_label, radio_uri);
  printf("TX %.0f МГц, RX %.0f МГц, %s, %d отсчетов на символ\n",
         transmit_frequency / 1e6, receive_frequency / 1e6, SELECTED_MODULATION == TX_QPSK ? "QPSK" : "BPSK", SYMBOL_SAMPLES);

  SoapySDRDevice_setSampleRate(radio, SOAPY_SDR_RX, 0, RADIO_SAMPLE_RATE);
  SoapySDRDevice_setSampleRate(radio, SOAPY_SDR_TX, 0, RADIO_SAMPLE_RATE);
  SoapySDRDevice_setFrequency(radio, SOAPY_SDR_RX, 0, receive_frequency, NULL);
  SoapySDRDevice_setFrequency(radio, SOAPY_SDR_TX, 0, transmit_frequency, NULL);

  size_t channel_ids[] = {0};
  SoapySDRDevice_setGain(radio, SOAPY_SDR_RX, *channel_ids, RECEIVER_GAIN);
  SoapySDRDevice_setGain(radio, SOAPY_SDR_TX, *channel_ids, TRANSMITTER_GAIN);
  size_t number_of_channels = sizeof(channel_ids) / sizeof(channel_ids[0]);

  SoapySDRStream *receiver_stream = SoapySDRDevice_setupStream(radio, SOAPY_SDR_RX, SOAPY_SDR_CS16, channel_ids, number_of_channels, NULL);
  SoapySDRStream *transmitter_stream = SoapySDRDevice_setupStream(radio, SOAPY_SDR_TX, SOAPY_SDR_CS16, channel_ids, number_of_channels, NULL);

  if (receiver_stream == NULL || transmitter_stream == NULL) {
    fprintf(stderr, "Не удалось создать SDR поток: %s\n", SoapySDRDevice_lastError());
    if (receiver_stream != NULL) SoapySDRDevice_closeStream(radio, receiver_stream);
    if (transmitter_stream != NULL) SoapySDRDevice_closeStream(radio, transmitter_stream);
    SoapySDRDevice_unmake(radio);
    return 1;
  }

  SoapySDRDevice_activateStream(radio, receiver_stream, 0, 0, 0);
  SoapySDRDevice_activateStream(radio, transmitter_stream, 0, 0, 0);

  size_t rx_block_size = SoapySDRDevice_getStreamMTU(radio, receiver_stream);
  size_t tx_block_size = SoapySDRDevice_getStreamMTU(radio, transmitter_stream);
  printf("MTU - TX: %zu, RX: %zu\n", tx_block_size, rx_block_size);
  int16_t rx_data[2 * rx_block_size];
  int16_t tx_data[2 * tx_block_size];
  int16_t file_buffer[2 * tx_block_size];

  const long read_timeout_us = 400000;
  long long previous_time_ns = 0;

  char capture_filename[64];
  snprintf(capture_filename, sizeof(capture_filename), "%s_sent.pcm", mode_label);
  FILE *capture_file = fopen(capture_filename, "wb");
  size_t block_index = 0;

  // подготовка передаваемого блока
  fill_transmit_frame(tx_data, tx_block_size, SELECTED_MODULATION);

  for (size_t i = 0; i < 2; i++) {
    tx_data[0 + i] = 0xffff;
    tx_data[10 + i] = 0xffff;
  }

  signal(SIGINT, stop_on_interrupt);

  void *input_buffers[] = {rx_data};
  // Удаляем старые буферы из очереди приёмника.
  for (size_t n = 0; n < RX_WARMUP_BUFFERS && work_active; ) {
    int io_flags;
    long long receive_time_ns;
    int read_count = SoapySDRDevice_readStream(radio, receiver_stream, input_buffers, rx_block_size, &io_flags, &receive_time_ns, read_timeout_us);
    if (read_count <= 0) continue;
    ++n;
  }

  printf("Start...\n");
  int max_amplitude = 0; // максимум за текущий интервал отчёта
  std::complex<double> phase_product_sum = 0;

  while (work_active) {
    int io_flags;
    long long receive_time_ns = previous_time_ns;
    int read_count = SoapySDRDevice_readStream(radio, receiver_stream, input_buffers, rx_block_size, &io_flags, &receive_time_ns, read_timeout_us);
    if (read_count <= 0) continue;

    // При пустом чтении не обновляем ни статистику, ни время передачи.
    // сбор статистики по входному потоку
    for (int i = 0; i < 2 * read_count; i++)
      if (abs(rx_data[i]) > max_amplitude)
        max_amplitude = abs(rx_data[i]);

    for (int i = 1; i < read_count; i++) {
      std::complex<double> present_sample (rx_data[2 * i], rx_data[2 * i + 1]);
      std::complex<double> last_sample(rx_data[2 * (i - 1)], rx_data[2 * (i - 1) + 1]);
      phase_product_sum += present_sample * std::conj(last_sample);
    }

    if (block_index % STATUS_PERIOD == 0) {
      double frequency_error_khz = std::arg(phase_product_sum) * RADIO_SAMPLE_RATE / (2.0 * M_PI) / 1e3;
      printf("Buffer: %zu - Time: %lli | принято: пик %4d из 2047, сдвиг f ~%+.2f кГц%s\n",
             block_index, receive_time_ns - previous_time_ns, max_amplitude, frequency_error_khz,
             max_amplitude < NOISE_THRESHOLD ? "  - это шум" : "");
      max_amplitude = 0;
      phase_product_sum = 0;
    }
    previous_time_ns = receive_time_ns;

    // планирование передачи относительно времени приёма
    long long scheduled_time_ns = previous_time_ns + TX_ADVANCE_NS;

    for (size_t i = 0; i < 8; i++) {
      const uint8_t timestamp_part = static_cast<uint8_t>((scheduled_time_ns >> (i * 8)) & 0xff);
      tx_data[2 + i] = timestamp_part << 4;
    }

    if (capture_file != NULL) {
      size_t values_written = 0;
      for (size_t i = 12; i < 2 * tx_block_size; i++)
        file_buffer[values_written++] = tx_data[i] / 16;
      fwrite(file_buffer, sizeof(int16_t), values_written, capture_file);
    }

    void *output_buffers[] = {tx_data};
    io_flags = SOAPY_SDR_HAS_TIME;
    int write_count = SoapySDRDevice_writeStream(radio, transmitter_stream, (const void * const*)output_buffers, tx_block_size, &io_flags, scheduled_time_ns, read_timeout_us);
    if ((size_t)write_count != tx_block_size) {
      printf("TX Failed: %i\n", write_count);
    }

    block_index++;
  }

  printf("\nОстановка... Записан %s\n", capture_filename);
  if (capture_file != NULL) fclose(capture_file);
  SoapySDRDevice_deactivateStream(radio, receiver_stream, 0, 0);
  SoapySDRDevice_deactivateStream(radio, transmitter_stream, 0, 0);
  SoapySDRDevice_closeStream(radio, receiver_stream);
  SoapySDRDevice_closeStream(radio, transmitter_stream);
  SoapySDRDevice_unmake(radio);
  return 0;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    printf("Использовать так: %s <rx|tx> [uri]\n", argv[0]);
    return 1;
  }
  const char *radio_uri="usb:0.1.5";
  if (argc > 2) {radio_uri=argv[2];}
  if (strcmp(argv[1], "rx") == 0){return start_radio(radio_uri, false);}
  else if (strcmp(argv[1], "tx") == 0) {return start_radio(radio_uri, true);}
  else {
    printf("Ошибка режима работы\n");
    return 1;
  }
}