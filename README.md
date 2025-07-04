# ArduinoChocadeira
Projetos de desenvolvimento C++
Com certeza! Para adaptar o código de controle da pastilha Peltier para uma chocadeira, precisamos ajustar a lógica do PID para aquecimento e redefinir as variáveis de temperatura para o contexto de uma incubadora.

Aqui está o código modificado para uma chocadeira (incubadora) usando a pastilha Peltier para aquecimento.

As principais mudanças são:

Nomenclatura de Pinos e Variáveis: Renomeei as variáveis para refletir melhor o uso em uma chocadeira (ex: incubatorTempPin para a temperatura interna da chocadeira, heatSinkTempPin para a temperatura do dissipador de calor da Peltier).

Faixa de Temperatura Alvo: O potenciômetro (potPin) agora mapeia a temperatura alvo para uma faixa mais apropriada para chocadeiras (36 a 38 graus Celsius).

Lógica PID para Aquecimento:

O cálculo do error (targetTemp - currentIncubatorTemp) agora gera um valor positivo quando a temperatura da chocadeira está abaixo do alvo, indicando a necessidade de aquecimento.

O termo Derivativo (D) agora é subtraído da soma P + I (PID_output = P + I - D;). Isso ocorre porque um aumento rápido na temperatura (currentIncubatorTemp - lastIncubatorTemp positivo) deve reduzir a saída do PID para evitar o "overshoot" (passar muito do alvo), atuando como um "freio".

O termo Integral (I) é limitado para não ser negativo, já que estamos apenas aquecendo.

Se a temperatura estiver no alvo ou acima (error <= 0), a saída do PID é zerada, desligando o aquecimento.

Segurança do Dissipador de Calor: A variável MAX_HEAT_SINK_TEMP foi aumentada para um valor mais realista (60.0°C) para o dissipador de calor da Peltier quando ela está aquecendo. Se o dissipador esquentar demais, a Peltier é desligada para protegê-la.

Mensagens do LCD e Serial: As mensagens foram atualizadas para refletir o contexto de uma chocadeira.

Função de Leitura de Temperatura: Criei uma função readTemperature para simplificar a leitura de ambos os termistores.

Código Arduino para Chocadeira Peltier
3/07, 15:04
Próximos Passos e Considerações Importantes:

Conexão da Pastilha Peltier para Aquecimento:

Para usar a Peltier para aquecimento, você precisará de um driver de ponte H (como um módulo L298N) ou um circuito de MOSFETs para controlar a direção da corrente. O pino peltierPin (10) do Arduino será conectado à entrada PWM do driver da ponte H. A ponte H será configurada para fornecer corrente à Peltier na direção que a faz aquecer o interior da chocadeira.

A Peltier deve ter um dissipador de calor adequado no lado "frio" (o lado que estará para fora da chocadeira) para dissipar o calor que ela está "bombeando" para fora do ambiente. Sem um bom dissipador, a Peltier pode superaquecer e ser danificada.

Ajuste dos Coeficientes PID (Tuning):

Os valores de Kp, Ki, e Kd (20.0, 0.5, 1.0) são apenas um ponto de partida. Você precisará "tuná-los" para o seu sistema específico. Isso envolve testar o comportamento da chocadeira e ajustar esses valores para obter uma temperatura estável, sem grandes oscilações e que atinja o alvo rapidamente.

Um bom método para começar é o método Ziegler-Nichols ou simplesmente ajustar manualmente:

Comece com Ki = 0 e Kd = 0.

Aumente Kp gradualmente até que a temperatura comece a oscilar em torno do ponto de ajuste.

Adicione Ki para eliminar o erro de estado estacionário (a diferença persistente entre a temperatura alvo e a real).

Adicione Kd para amortecer as oscilações e melhorar a resposta a mudanças rápidas.

Localização dos Sensores:

O incubatorTempPin (A1) deve estar posicionado dentro da chocadeira, em um local que represente bem a temperatura média dos ovos, longe de correntes de ar diretas ou da própria pastilha Peltier.

O heatSinkTempPin (A0) deve estar no dissipador de calor da Peltier (o lado que fica para fora da chocadeira) para monitorar sua temperatura de segurança.

Isolamento da Chocadeira:

Um bom isolamento da chocadeira é fundamental para a eficiência da Peltier e para a estabilidade da temperatura.

Umidade e Viragem de Ovos (Recursos Adicionais):

Uma chocadeira real geralmente também requer controle de umidade e um mecanismo para virar os ovos periodicamente. Se desejar adicionar essas funcionalidades, podemos expandir o código.

Este código fornece uma base sólida para o controle de temperatura da sua chocadeira. Lembre-se que o "tuning" do PID é a parte mais importante para o desempenho ideal.

/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\\/

Olá! Entendido. Você quer que o potenciômetro forneça valores com casas decimais no Arduino.

No seu código atual, a linha que lê o potenciômetro e define a temperatura alvo é:

C++

targetTemp = map(analogRead(potPin), 0, 1023, 36, 38);
A função map() do Arduino é muito útil, mas ela trabalha apenas com números inteiros. Isso significa que, mesmo que targetTemp seja do tipo double, o resultado da função map() será sempre um inteiro, e as casas decimais serão perdidas (ou seja, 36.0, 37.0, 38.0, mas nunca 36.5 ou 37.2).

Para obter valores com casas decimais a partir do potenciômetro, você precisa realizar o mapeamento manualmente, usando aritmética de ponto flutuante.

Aqui está como você pode fazer isso, substituindo a linha map():

C++

// Leitura do valor analógico bruto do potenciômetro (0 a 1023)
double rawPotValue = analogRead(potPin);

// Mapeia o valor bruto para a faixa desejada (36.0 a 38.0) com casas decimais
// Usamos 1023.0 (com ponto decimal) para garantir que a divisão seja de ponto flutuante
targetTemp = 36.0 + (rawPotValue / 1023.0) * (38.0 - 36.0);
Explicação:

double rawPotValue = analogRead(potPin);: Lê o valor bruto do potenciômetro (um inteiro de 0 a 1023) e o armazena como um double. Isso não é estritamente necessário aqui, mas garante que a próxima operação seja de ponto flutuante.

rawPotValue / 1023.0: Calcula a proporção da leitura do potenciômetro em relação ao seu valor máximo. Ao usar 1023.0 (com o .0), forçamos a divisão a ser de ponto flutuante, resultando em um valor como 0.5, 0.75, etc., em vez de um inteiro (0 ou 1).

(38.0 - 36.0): Calcula a amplitude da sua faixa de temperatura alvo (neste caso, 2.0).

(rawPotValue / 1023.0) * (38.0 - 36.0): Multiplica a proporção pela amplitude da faixa. Isso dá a "distância" dentro da sua faixa.

36.0 + ...: Adiciona o valor mínimo da sua faixa (36.0) ao resultado, deslocando-o para a faixa correta.

Ao fazer essa alteração, sua variável targetTemp terá agora valores com casas decimais, permitindo um ajuste mais fino da temperatura alvo na sua chocadeira.

/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

double potpin = 0;  // analog pin used to connect the potentiometer
double val;    // variable to read the value from the analog pin

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

void loop() {
  double valor = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = 0.0 + (valor / 1023.0) * (180.0 - 0.0);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);
  Serial.println(val);                           // waits for the servo to get there;
}

/*
double rawPotValue = analogRead(potPin);

// Mapeia o valor bruto para a faixa desejada (36.0 a 38.0) com casas decimais
// Usamos 1023.0 (com ponto decimal) para garantir que a divisão seja de ponto flutuante
targetTemp = 36.0 + (rawPotValue / 1023.0) * (38.0 - 36.0);
Explicação:

double rawPotValue = analogRead(potPin);: Lê o valor bruto do potenciômetro (um inteiro de 0 a 1023) e o armazena como um double. Isso não é estritamente necessário aqui, mas garante que a próxima operação seja de ponto flutuante.

rawPotValue / 1023.0: Calcula a proporção da leitura do potenciômetro em relação ao seu valor máximo. Ao usar 1023.0 (com o .0), forçamos a divisão a ser de ponto flutuante, resultando em um valor como 0.5, 0.75, etc., em vez de um inteiro (0 ou 1).

(38.0 - 36.0): Calcula a amplitude da sua faixa de temperatura alvo (neste caso, 2.0).

(rawPotValue / 1023.0) * (38.0 - 36.0): Multiplica a proporção pela amplitude da faixa. Isso dá a "distância" dentro da sua faixa.

36.0 + ...: Adiciona o valor mínimo da sua faixa (36.0) ao resultado, deslocando-o para a faixa correta.

Ao fazer essa alteração, sua variável targetTemp terá agora valores com casas decimais, permitindo um ajuste mais fino da temperatura alvo na sua chocadeira.
*/
