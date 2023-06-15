#include "arduinoFFT.h"

//    Se guardan variables y se  
//    crean instancias para la TTF
#define SAMPLES 128
#define SAMPLING_FREQUENCY 3500

arduinoFFT FFT = arduinoFFT();

unsigned int sampling_period_us;
unsigned long microseconds;


double vReal[SAMPLES];
double vImag[SAMPLES];

//    Variables para la afinación

const double tunning[6] = {330.0, 252.5, 198.5, 149.5, 112.5, 84.0};    // arreglo con las frecuencias deseadas
short int string = 0;                                                   // cuerda a afinar


double tune = 0.0;          // variable que guarda la última frecuencia obtenida por la ttf
short int state;            // guarda el estado: 0 para seleccionar cuerda, 1 para encontrar la frecuencia,
                            // 2 para mover el motor


const short int pPin = 2;   // pines de salida para el motor
const short int nPin = 3;


const int vol_th = 575;     // volumen al que se empiezan a tomar medidas
const double tone_th = 0.75;      // error en la frecuencia máxima

short int prev_switch = 0;

//
//    FUNCIONES DE ARDUINO
//


void setup() {

  // Para la TTF
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  Serial.begin(115200);
  while(!Serial);

  // Serial.println("Ready");

  // Para seleccionar los pines
  pinMode(pPin, OUTPUT);
  pinMode(nPin, OUTPUT);

  // Se inicia en el primer estado 0
  state = 0;

  // Se seleccionan los pines de entrada
  // para la selección de cuerdas
  for (short int i = 8; i <= 10; i++)
  {
    pinMode(i, INPUT);
  }

}

void loop() {

  // Serial.println(state);

// según el estado, se hace una u otra cosa
  switch (state)
  {
    case 0:   

      //Serial.println("Seleccionando cuerda");                  

      for (int i = 8; i <= 10; i++)             // en el estado 0, se selecciona la cuerda a afinar
      {                                         // según el array de frecuencias deseadas y se pasa al estado 1.
        if (digitalRead(i) == HIGH)             // cuando el motor logró afinar la cuerda, se devuelve a este estado.
        {

            if (prev_switch == i)
            {
              break;
            }
            else
            {

              prev_switch = i;

              switch (i)
              {
                case 8:

                  if (string != 0){string--;} break;
                
                case 9:

                  if (string != 5){string++;} break;

                case 10:

                  state = 1; 
                  Serial.print("Cambio de estado: ");
                  Serial.println(state);
                  Serial.print("Cuerda elegida: ");
                  Serial.println(string);
                  break;
                
                default: void;
              }
            }

            delay(200);

        }
        else
        {
          prev_switch = 0;
        }
      }

      break;

    case 1:                               // este estado encuentra la frecuencia actual de la guitarra, para eso
      if (analogRead(A5) > vol_th)        // si el volumen pasa cierto umbral vol_th, 
      {
        tune = FindDominantFrequency();   // halla la frecuencia con la función FindDominantFrequency y almacena el resultado
        state = 2;                        // en tune, para que pueda ser usado por otras funciones. Pasa al estado 2.

        Serial.print("Cambio de estado: ");
        Serial.println(state);
      }; break;
    
    case 2:                                                   // este estado mueve el motor comparando la frecuencua real y la deseada
      if (move_string() == 1)
      {state = 0; string = 0;}
      else
      {state = 1;};                                           // a través de la función move_string(). Si el motor afinó la cuerda, 

      Serial.print("Cambio de estado: ");
      Serial.println(state);
      break;                                                  // se devuelve al estado 0, si no, al estado 1.

    default: void;    // por defecto, no se hace nada.
  }

}


//
//    FUNCIONES ESPECIALES DEL PROYECTO
//


// Función que halla la frecuencia instantánea de la guitarra
// y devuelve la frecuencia dominante.
double frec_it()
{
  for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();    
        vReal[i] = analogRead(A5);
        vImag[i] = 0;
        while(micros() < (microseconds + sampling_period_us));
    }
 
    /*FFT*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

    return FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY) * 0.990;
}


//  Función que decide cuál es la frecuencia real en cada instante,
//  busca que las últimas 3 frecuencias obtenidas estén en cierto rango
//  y devuelve su promedio.
double FindDominantFrequency()
{

  int counter = 0;                // se tiene un contador de cuántas veces se va obteniendo
                                  // cierta frecuencia.

  double last_val = frec_it();    // variable que guarda la frecuencia dominante en cierto instante.


  // se inicia el bucle que busca que las frecuencias estén dentro de
  // determinado rango.
  while (counter < 3)
  {

    // nueva variable para la frecuencia instantánea
    double inst_freq = frec_it();

    // si la diferencia entre las dos frecuencias está en un rango,
    // se aumenta 1 al contador y se almacena en last_val el promedio de ambas.
    if (abs(last_val - inst_freq) < tone_th)
    {
      counter++;
      last_val = (last_val + inst_freq) / 2;
    }
    else    // en caso contrario, se deja el contador en 0 y se vuelve a iniciar.
    {
      counter = 0;
      last_val = frec_it();
    }

  }

  // Cuando se hallan encontrado 3 frecuencias dentro del rango, se devuelve su promedio
  return last_val;
}


// Función que mueve la cuerda. Si logró afinarla, devuelve true (1),
// si no, devuelve false (0)
bool move_string()
{

  double diff = tune - tunning[string];   // se halla la diferencia entre la frecuencia esperada
                                          // y la frecuencia real.

  // debug             
  Serial.print(diff);
  Serial.print(", ");
  Serial.println(tune);


  // si esta frecuencia está muy por encima, retorna false.
  if (abs(diff) > 50.0)
  {
    return 0;
  }


  // si la diferencia entre frecuencias es menor a nuestro treshold,
  // la cuerda está afinada y se devuelve true.
  if ( abs(diff) < tone_th )
  {
    return 1;
  }


  // dependiendo si la diferencia fue negativa o positiva, 
  // se mueve el motor en una dirección u otra durante un tiempo
  // dependiente de cuánto fue esta diferencia.
  if (diff < 0)
  {
      digitalWrite(pPin, HIGH);

      delay(-1.0 * diff * 30.0);

      digitalWrite(pPin, LOW);
  }
  else
  {
    digitalWrite(nPin, HIGH);

      delay(1.0 * diff * 30.0);

      digitalWrite(nPin, LOW);
  }


  // se devuelve false, por consiguiente, el estado global se vuelve 1.
  return 0;
}
