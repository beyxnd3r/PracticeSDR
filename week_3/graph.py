
import numpy as np
import matplotlib.pyplot as plt

filename = "build/rxdata.pcm"

data = np.fromfile(filename, dtype=np.int16)

i_signal = data[::2]
q_signal = data[1::2]
samples = np.arange(len(i_signal))

plt.figure(figsize=(14, 5))
plt.plot(samples, i_signal, color='green', label='I Channel', alpha=0.8)
plt.plot(samples, q_signal, color='orange', label='Q Channel', alpha=0.8)

plt.title("Анализ принятого IQ-сигнала")
plt.xlabel("Индекс отсчёта")
plt.ylabel("Значение амплитуды")
plt.legend()
plt.grid(linestyle='--', alpha=0.5)
plt.tight_layout()
plt.show()
