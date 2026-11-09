
import numpy as np
import matplotlib.pyplot as plt

# Чтение IQ-отсчётов из файла
filename = "build/txdata.pcm"

samples = np.fromfile(filename, dtype=np.int16)

# Разделение на I и Q составляющие
i_data = samples[::2]
q_data = samples[1::2]
indexes = np.arange(len(i_data))

# Построение графика
plt.figure(figsize=(13, 5))

plt.plot(indexes, i_data, color='green', label='I Channel')
plt.plot(indexes, q_data, color='orange', label='Q Channel')

plt.title("Принятые IQ-данные")
plt.xlabel("Номер отсчёта")
plt.ylabel("Амплитуда")

plt.legend()
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()
plt.show()
