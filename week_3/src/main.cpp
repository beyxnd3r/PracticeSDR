
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main(void)
{
    // Подключение к PlutoSDR
    SoapySDRKwargs config = {};
    SoapySDRKwargs_set(&config, "driver", "plutosdr");
    SoapySDRKwargs_set(&config, "uri", "usb:");

    SoapySDRDevice *device = SoapySDRDevice_make(&config);
    SoapySDRKwargs_clear(&config);

    if (device == NULL)
    {
        fprintf(stderr, "Ошибка подключения SDR: %s\n",
                SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    // Установка параметров приемника
    double sample_rate = 1000000;
    double frequency = 800000000;
    double gain = 40.0;

    SoapySDRDevice_setSampleRate(device, SOAPY_SDR_RX, 0, sample_rate);
    SoapySDRDevice_setFrequency(device, SOAPY_SDR_RX, 0, frequency, NULL);
    SoapySDRDevice_setGain(device, SOAPY_SDR_RX, 0, gain);

    // Создание потока для приема
    size_t channel_list[] = {0};

    SoapySDRStream *stream = SoapySDRDevice_setupStream(
        device, SOAPY_SDR_RX, SOAPY_SDR_CS16,
        channel_list, 1, NULL);

    if (stream == NULL)
    {
        fprintf(stderr, "Не удалось создать поток: %s\n",
                SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(device);
        return EXIT_FAILURE;
    }

    SoapySDRDevice_activateStream(device, stream, 0, 0, 0);

    // Размер буфера
    size_t buffer_size = SoapySDRDevice_getStreamMTU(device, stream);
    printf("Размер буфера RX: %zu\n", buffer_size);

    int16_t *iq_buffer = malloc(2 * buffer_size * sizeof(int16_t));

    if (iq_buffer == NULL)
    {
        printf("Ошибка выделения памяти\n");
        SoapySDRDevice_deactivateStream(device, stream, 0, 0);
        SoapySDRDevice_closeStream(device, stream);
        SoapySDRDevice_unmake(device);
        return EXIT_FAILURE;
    }

    FILE *output = fopen("rxdata.pcm", "wb");

    if (output == NULL)
    {
        printf("Ошибка создания файла\n");
        free(iq_buffer);
        SoapySDRDevice_deactivateStream(device, stream, 0, 0);
        SoapySDRDevice_closeStream(device, stream);
        SoapySDRDevice_unmake(device);
        return EXIT_FAILURE;
    }

    printf("Запись IQ-данных началась...\n");

    // Считывание отсчетов
    const int max_reads = 100;

    for (int n = 0; n < max_reads; n++)
    {
        void *buffers[] = {iq_buffer};
        int flags = 0;
        long long timestamp = 0;

        int received = SoapySDRDevice_readStream(
            device, stream, buffers, buffer_size,
            &flags, &timestamp, 1000000);

        if (received > 0)
        {
            fwrite(iq_buffer, sizeof(int16_t),
                   2 * received, output);
        }
        else if (received < 0)
        {
            fprintf(stderr, "Ошибка RX: %s\n",
                    SoapySDRDevice_lastError());
        }
    }

    fclose(output);
    free(iq_buffer);

    // Остановка и освобождение ресурсов
    SoapySDRDevice_deactivateStream(device, stream, 0, 0);
    SoapySDRDevice_closeStream(device, stream);
    SoapySDRDevice_unmake(device);

    printf("Данные записаны в rxdata.pcm\n");

    return EXIT_SUCCESS;
}
