\# Análise de Vibração com STM32H755 e MPU6050



Projeto de processamento digital de sinais desenvolvido com a placa \*\*NUCLEO-H755ZI-Q\*\* e o acelerômetro \*\*MPU6050\*\*, com foco em aquisição de sinais de vibração e análise no domínio da frequência utilizando FFT.



O objetivo do projeto é estudar, de forma prática, conceitos de sistemas embarcados e processamento digital de sinais, desde a aquisição dos dados do sensor até a identificação das principais componentes de frequência presentes no sinal.



\## Funcionalidades



O projeto atualmente realiza:



\- Comunicação com o MPU6050 através de I2C;

\- Verificação do sensor através do registrador `WHO\_AM\_I`;

\- Leitura dos eixos X, Y e Z do acelerômetro;

\- Amostragem periódica utilizando o TIM7;

\- Armazenamento das amostras em buffer;

\- Filtro de média móvel;

\- Remoção da componente DC;

\- Aplicação de janela de Hann;

\- FFT de 256 pontos;

\- Cálculo da magnitude do espectro;

\- Identificação do bin de maior magnitude;

\- Conversão do bin dominante para frequência em Hz.



\## Hardware utilizado



\- STM32 NUCLEO-H755ZI-Q;

\- MPU6050 / GY-521;

\- Comunicação I2C;

\- ST-Link integrado da placa.



\## Software e bibliotecas



\- STM32CubeIDE;

\- STM32 HAL;

\- CMSIS-DSP;

\- Linguagem C.



\## Processamento do sinal



O fluxo simplificado do projeto é:



```text

MPU6050

&#x20;  |

&#x20;  v

Leitura I2C

&#x20;  |

&#x20;  v

Amostragem com TIM7

&#x20;  |

&#x20;  v

Buffer de 256 amostras

&#x20;  |

&#x20;  v

Remoção da média

&#x20;  |

&#x20;  v

Janela de Hann

&#x20;  |

&#x20;  v

FFT

&#x20;  |

&#x20;  v

Magnitude do espectro

&#x20;  |

&#x20;  v

Detecção da frequência dominante

```



\## FFT



A FFT é realizada utilizando a biblioteca CMSIS-DSP:



```c

arm\_rfft\_fast\_init\_f32(\&fft\_instance, FFT\_SIZE);

```



O tamanho utilizado atualmente é:



```c

\#define FFT\_SIZE 256

```



Antes da FFT, a média das amostras é calculada e removida para reduzir a componente DC.



Em seguida, é aplicada uma janela de Hann para reduzir o vazamento espectral.



```c

hann\_window\[i] =

&#x20;   0.5f \*

&#x20;   (1.0f -

&#x20;   cosf((2.0f \* PI \* i) / (FFT\_SIZE - 1)));

```



Depois disso, a FFT é calculada:



```c

arm\_rfft\_fast\_f32(

&#x20;   \&fft\_instance,

&#x20;   ax\_fft\_buffer,

&#x20;   fft\_output,

&#x20;   0

);

```



A magnitude das componentes do espectro é obtida utilizando:



```c

arm\_cmplx\_mag\_f32();

```



O algoritmo então procura o bin com maior magnitude e calcula a frequência correspondente.



\## Aquisição do MPU6050



O MPU6050 é acessado utilizando I2C.



Endereço utilizado:



```c

\#define MPU6050\_ADDR (0x68 << 1)

```



Antes da aquisição dos dados, o firmware verifica o registrador:



```c

\#define MPU6050\_WHO\_AM\_I 0x75

```



O valor esperado é:



```text

0x68

```



Os dados dos três eixos do acelerômetro são lidos a partir do registrador:



```c

\#define MPU6050\_ACCEL\_XOUT 0x3B

```



São lidos seis bytes:



```text

AX HIGH

AX LOW

AY HIGH

AY LOW

AZ HIGH

AZ LOW

```



\## Estrutura do projeto



```text

analise-vibracao-stm32-mpu6050/

│

├── CM4/

├── CM7/

├── Common/

├── Drivers/

│

├── dsp\_acelerometro.ioc

├── .project

├── .mxproject

└── README.md

```



O processamento principal do sinal está atualmente sendo desenvolvido no Cortex-M7.



\## Objetivos de aprendizado



Este projeto também serve como estudo prático de:



\- Microcontroladores STM32;

\- Programação embarcada em C;

\- Comunicação I2C;

\- Timers e interrupções;

\- Aquisição de sensores;

\- Processamento digital de sinais;

\- FFT;

\- Análise espectral;

\- CMSIS-DSP;

\- Detecção de vibrações.



\## Próximos passos



Algumas melhorias planejadas para o projeto incluem:



\- Conversão das leituras do acelerômetro para unidades físicas;

\- Ajuste e validação da taxa real de amostragem;

\- Melhor análise das magnitudes da FFT;

\- Detecção automática de diferentes padrões de vibração;

\- Extração de características do sinal;

\- Envio das características para outro sistema;

\- Avaliação do uso de técnicas de TinyML para classificação de padrões de vibração.



\## Autor



\*\*Ian Godoi\*\*



Graduando em Ciência da Computação pela Universidade Federal de Minas Gerais — UFMG.



