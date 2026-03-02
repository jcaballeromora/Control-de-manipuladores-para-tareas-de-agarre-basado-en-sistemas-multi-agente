# Control de Manipuladores para Tareas de Agarre basado en Sistemas Multi-Agente (MARS)

Este repositorio contiene el código fuente, modelos de simulación y documentación técnica de la tesis doctoral enfocada en la manipulación cooperativa utilizando brazos industriales **Universal Robots (UR3)**.



## 📝 Resumen
Esta investigación propone un esquema de control híbrido que integra la técnica de **enjaulamiento cooperativo (caging)** con la regulación dinámica de fuerza mediante **control de admitancia**. El sistema permite que múltiples manipuladores coordinen sus movimientos para asegurar la integridad de un objeto durante su transporte. 

Un aporte principal es la superación de las limitaciones de hardware de la serie **UR3 CB-Series** (que carece de sensores de fuerza nativos) mediante una arquitectura abierta asíncrona en Python, utilizando estimación indirecta de fuerza y filtrado digital avanzado.

## 🚀 Características Principales
* **Arquitectura Abierta:** Implementación de comunicación asíncrona mediante sockets TCP/IP y protocolo RTDE.
* **Sistemas Multi-Agente (MARS):** Algoritmos basados en Funciones de Potencial Artificial (APF) para la formación y evasión de colisiones.
* **Control de Admitancia:** Algoritmo discreto para la regulación de fuerza de contacto sin transductores externos.
* **Procesamiento de Señal:** Filtrado digital IIR de primer orden para mitigar el ruido en la estimación de corriente.
* **Soporte Heterogéneo:** Validación experimental en plataformas UR3 CB-Series y e-Series.



## 🛠️ Estructura del Repositorio
* `/src`: Scripts de control en Python (lógica de admitancia, MARS y comunicación).
* `/sim`: Entornos de simulación en RoboDK y validaciones en MATLAB/Simscape.
* `/docs`: Documentación técnica, diagramas de flujo y manuales.
* `/certificados`: Certificaciones oficiales de Universal Robots Academy (Core, Pro, Advanced).

## 📋 Requisitos
* Python 3.8+
* Librerías: `numpy`, `matplotlib`, `urx` o `dashboard_client`.
* Hardware: Universal Robots UR3 (CB3 o e-Series) con puerto Ethernet habilitado.

## 📊 Resultados
El sistema logra una regulación de fuerza estable con una consigna de **20 N**, compensando eficazmente las latencias de red y manteniendo una formación asintóticamente estable en tareas de manipulación cooperativa.

## 🎓 Autor
**Julio Antonio Caballero Mora** Doctorado en Ingeniería Aplicada / Sistemas Mecatrónicos  
Universidad Veracruzana, México.

## 📄 Cita (Citation)
Si utilizas este código o los conceptos de esta investigación en tu trabajo académico, por favor cítalo de la siguiente manera:

```bibtex
@phdthesis{CaballeroMora2026,
  author = {Julio Antonio Caballero Mora},
  title = {Control de Manipuladores para Tareas de Agarre basado en Sistemas Multi-Agente},
  school = {Universidad Veracruzana},
  year = {2026},
  address = {Veracruz, México},
  type = {Tesis Doctoral}
}
