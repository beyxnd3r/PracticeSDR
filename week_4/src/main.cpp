
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main(void)
{
    // Подключение SDR-устройства
    SoapySDRKwargs config = {};
    SoapySDRKwargs_set(&config, "driver", "plutosdr");
    SoapySDRKwargs_set(&config, "uri", "usb:");
    SoapySDRKwargs_set(&config, "direct", "1");
    SoapySDRKwargs_set(&config, "timestamp_every", "1920");
    SoapySDRKwargs_set(&config, "loopback", "0");

    SoapySDRDevice *device = SoapySDRDevice_make(&config);
    SoapySDRKwargs_clear(&config);

    if (device == NULL)
    {
        printf("SDR connection error: %s\n",
               SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    // Настройка частоты и скорости RX/TX
    double sample_rate = 1e6;
    double center_freq = 800e6;

    if (SoapySDRDevice_setSampleRate(device, SOAPY_SDR_RX, 0, sample_rate) != 0)
        printf("RX sample rate error\n");

    if (SoapySDRDevice_setFrequency(device, SOAPY_SDR_RX, 0, center_freq, NULL) != 0)
        printf("RX frequency error\n");

    if (SoapySDRDevice_setSampleRate(device, SOAPY_SDR_TX, 0, sample_rate) != 0)
        printf("TX sample rate error\n");

    if (SoapySDRDevice_setFrequency(device, SOAPY_SDR_TX, 0, center_freq, NULL) != 0)
        printf("TX frequency error\n");

    printf("TX frequency: %.0f Hz\n",
           SoapySDRDevice_getFrequency(device, SOAPY_SDR_TX, 0));

    // Выбор рабочего канала
    size_t channels[] = {0};
    size_t num_channels = sizeof(channels) / sizeof(channels[0]);

    SoapySDRDevice_setGain(device, SOAPY_SDR_RX, 0, 30.0);
    SoapySDRDevice_setGain(device, SOAPY_SDR_TX, 0, -40.0);

    // Создание потоков
    SoapySDRStream *rx = SoapySDRDevice_setupStream(
        device, SOAPY_SDR_RX, SOAPY_SDR_CS16,
        channels, num_channels, NULL);

    if (rx == NULL)
    {
        printf("RX stream error: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(device);
        return EXIT_FAILURE;
    }

    SoapySDRStream *tx = SoapySDRDevice_setupStream(
        device, SOAPY_SDR_TX, SOAPY_SDR_CS16,
        channels, num_channels, NULL);

    if (tx == NULL)
    {
        printf("TX stream error: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_closeStream(device, rx);
        SoapySDRDevice_unmake(device);
        return EXIT_FAILURE;
    }

    // Определение размеров буферов
    size_t rx_size = SoapySDRDevice_getStreamMTU(device, rx);
    size_t tx_size = SoapySDRDevice_getStreamMTU(device, tx);

    printf("RX MTU: %zu | TX MTU: %zu\n", rx_size, tx_size);

    int16_t *rx_samples = malloc(2 * rx_size * sizeof(int16_t));
    int16_t *tx_samples = malloc(2 * tx_size * sizeof(int16_t));

    if (rx_samples == NULL || tx_samples == NULL)
    {
        printf("Buffer allocation failed\n");
        free(rx_samples);
        free(tx_samples);
        SoapySDRDevice_closeStream(device, rx);
        SoapySDRDevice_closeStream(device, tx);
        SoapySDRDevice_unmake(device);
        return EXIT_FAILURE;
    }

    // Заполнение передающего буфера
    for (size_t i = 0; i < tx_size; i++)
    {
        tx_samples[2 * i] = 2000;
        tx_samples[2 * i + 1] = 2000;
    }

    // Добавление служебных маркеров
    if (tx_size >= 6)
    {
        tx_samples[0] = (int16_t)0xffff;
        tx_samples[1] = (int16_t)0xffff;
        tx_samples[10] = (int16_t)0xffff;
        tx_samples[11] = (int16_t)0xffff;
    }

    FILE *output = fopen("rxdata.pcm", "wb");

    if (output == NULL)
    {
        printf("Cannot create output file\n");
        free(rx_samples);
        free(tx_samples);
        SoapySDRDevice_closeStream(device, rx);
        SoapySDRDevice_closeStream(device, tx);
        SoapySDRDevice_unmake(device);
        return EXIT_FAILURE;
    }

    // Активация потоков
    int rx_status = SoapySDRDevice_activateStream(device, rx, 0, 0, 0);
    int tx_status = SoapySDRDevice_activateStream(device, tx, 0, 0, 0);

    if (rx_status != 0 || tx_status != 0)
    {
        printf("Stream activation failed\n");
        fclose(output);
        free(rx_samples);
        free(tx_samples);
        SoapySDRDevice_closeStream(device, rx);
        SoapySDRDevice_closeStream(device, tx);
        SoapySDRDevice_unmake(device);
        return EXIT_FAILURE;
    }

    printf("Starting RX/TX experiment...\n");

    const long timeout_us = 400000;
    const size_t total_cycles = 10;

    long long previous_time = 0;

    // Основной цикл обработки
    for (size_t cycle = 0; cycle < total_cycles; cycle++)
    {
        void *rx_buffers[] = {rx_samples};

        int flags = 0;
        long long rx_time = 0;

        int received = SoapySDRDevice_readStream(
            device, rx, rx_buffers, rx_size,
            &flags, &rx_time, timeout_us);

        if (received <= 0)
        {
            printf("RX skipped at cycle %zu\n", cycle);
            continue;
        }

        printf("Cycle %zu: samples=%d, time=%lld, delta=%lld\n",
               cycle, received, rx_time,
               rx_time - previous_time);

        previous_time = rx_time;

        // Сохранение полученных IQ-данных
        fwrite(rx_samples, sizeof(int16_t),
               2 * received, output);

        // Планируем передачу через 4 мс
        long long scheduled_time = rx_time + 4000000LL;

        // Записываем временную метку в TX-буфер
        if (tx_size >= 6)
        {
            for (size_t k = 0; k < 8; k++)
            {
                uint8_t part = (scheduled_time >> (8 * k)) & 0xff;
                tx_samples[2 + k] = (int16_t)(part << 4);
            }
        }

        // Передача на третьей итерации
        if (cycle == 2)
        {
            const void *tx_buffers[] = {tx_samples};
            int tx_flags = SOAPY_SDR_HAS_TIME;

            int sent = SoapySDRDevice_writeStream(
                device, tx, tx_buffers, tx_size,
                &tx_flags, scheduled_time, timeout_us);

            if (sent < 0)
                printf("TX error: %s\n", SoapySDRDevice_lastError());
            else
                printf("TX sent: %d / %zu samples\n", sent, tx_size);
        }
    }

    fclose(output);

    // Завершение работы потоков
    SoapySDRDevice_deactivateStream(device, rx, 0, 0);
    SoapySDRDevice_deactivateStream(device, tx, 0, 0);

    SoapySDRDevice_closeStream(device, rx);
    SoapySDRDevice_closeStream(device, tx);

    free(rx_samples);
    free(tx_samples);

    SoapySDRDevice_unmake(device);

    printf("Experiment completed. IQ data saved.\n");

    return EXIT_SUCCESS;
}
