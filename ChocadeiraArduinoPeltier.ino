// Inclusões de bibliotecas necessárias
#include <TimerOne.h>
#include <ACI_10K_an.h> // Para conversão precisa do termistor NTC 10K
#include <LiquidCrystal.h> // Para display LCD

// ============================================================================
// Definições de Pinos
// ============================================================================
const int incubatorTempPin = A1; // Pino do Termistor dentro da chocadeira (temperatura controlada)
const int heatSinkTempPin = A0;  // Pino do Termistor no dissipador de calor (lado "frio" da Peltier ao aquecer)
const int peltierPin = 10;       // Pino PWM para controle da Pastilha Peltier (Pin 10 é controlado pelo Timer1)
double potPin = A4;           // Pino do Potenciômetro para ajuste da temperatura alvo

// Configuração dos pinos do display LCD (RS, Enable, D4, D5, D6, D7)
LiquidCrystal lcd(12, 11, 9, 8, 7, 6);

// ============================================================================
// Instâncias de Sensores
// ============================================================================
Aci_10K heatSinkTermistor;   // Instância para o termistor do dissipador de calor
Aci_10K incubatorTermistor;  // Instância para o termistor dentro da chocadeira

// ============================================================================
// Variáveis Globais de Temperatura
// ============================================================================
double currentIncubatorTemp; // Temperatura medida dentro da chocadeira
double currentHeatSinkTemp;  // Temperatura medida no dissipador de calor (lado "frio" da Peltier ao aquecer)
double targetTemp;           // Temperatura alvo definida pelo potenciômetro

// ============================================================================
// Variáveis do Controlador PID
// ============================================================================
double P = 0.0, I = 0.0, D = 0.0, PID_output = 0; // Termos P, I, D e a saída final do PID
// Coeficientes PID iniciais (AJUSTE CONFORME NECESSÁRIO PARA AQUECIMENTO)
// Estes valores são um ponto de partida e precisarão ser ajustados ("tunados")
// para o seu sistema físico específico (tamanho da chocadeira, isolamento, potência da Peltier).
double Kp = 20.0, Ki = 0.5, Kd = 1.0;             

double lastIncubatorTemp = 0;    // Última temperatura da chocadeira para o cálculo do termo Derivativo (D)
double error = 0;                // Erro (diferença entre temperatura alvo e temperatura atual)
unsigned long lastProcessTime = 0; // Armazena o tempo do último cálculo do PID
const unsigned long PID_INTERVAL = 1000; // Intervalo de 1 segundo para o ciclo de cálculo do PID

// ============================================================================
// Variáveis de Segurança (Temperatura do Dissipador de Calor)
// ============================================================================
unsigned long lastHeatSinkCheckTime = 0; // Armazena o tempo da última verificação do dissipador
const unsigned long HEAT_SINK_CHECK_INTERVAL = 2000; // Verifica o dissipador a cada 2 segundos
double MAX_HEAT_SINK_TEMP; // Temperatura máxima permitida para o dissipador (segurança)
const double HEAT_SINK_HYSTERESIS = 5.0; // Histerese para reativar a Peltier (ex: 60.0 - 5.0 = 55.0)

// Flag para indicar se a Peltier foi desativada devido à alta temperatura do dissipador
bool peltier_disabled_by_safety = false;

// ============================================================================
// Protótipos de Funções
// ============================================================================
float readTemperature(int pin, Aci_10K& termistorInstance); // Função para ler temperatura de um termistor
void printDebugInfo(); // Imprime informações no Serial Monitor
void updateLCD();      // Atualiza o display LCD

// ============================================================================
// Função Setup
// Inicializa comunicação serial, LCD, pinos e timers.
// ============================================================================
void setup() {
    Serial.begin(9600); // Inicia a comunicação serial para depuração
    Serial.println("Sistema de Chocadeira Peltier Iniciado!");

    lcd.begin(16, 2); // Inicializa o display LCD (16 colunas, 2 linhas)
    lcd.print("Chocadeira ON!"); // Mensagem inicial no LCD

    // Configura os pinos como entrada ou saída
    pinMode(incubatorTempPin, INPUT);
    pinMode(heatSinkTempPin, INPUT);
    pinMode(potPin, INPUT);
    pinMode(peltierPin, OUTPUT);

    // Lê as temperaturas iniciais e define 'lastIncubatorTemp' para um cálculo de D válido
    currentIncubatorTemp = readTemperature(incubatorTempPin, incubatorTermistor);
    currentHeatSinkTemp = readTemperature(heatSinkTempPin, heatSinkTermistor);
    // Leitura do valor analógico bruto do potenciômetro (0 a 1023)
    double rawPotValue = analogRead(potPin);

    // Mapeia o valor bruto para a faixa desejada (36.0 a 38.0) com casas decimais
    // Usamos 1023.0 (com ponto decimal) para garantir que a divisão seja de ponto flutuante
    targetTemp = 36.0 + (rawPotValue / 1023.0) * (38.0 - 36.0);// Mapeia para faixa de chocadeira (36 a 38 C)
    lastIncubatorTemp = currentIncubatorTemp; // Essencial para o cálculo correto do termo Derivativo (D)

    // Inicializa o Timer1 para gerar um sinal PWM no pino da Peltier
    // A frequência de 1kHz (1000us) é um bom ponto de partida para PWM.
    Timer1.initialize(1000);
    Timer1.pwm(peltierPin, 0); // Garante que a Peltier comece desligada (0% de ciclo de trabalho)

    lastProcessTime = millis(); // Inicializa o tempo para o primeiro ciclo do PID
}

// ============================================================================
// Função Loop Principal
// É executada continuamente, controlando o sistema de climatização.
// ============================================================================
void loop() {
    unsigned long currentTime = millis(); // Obtém o tempo atual em milissegundos
    
    // 1. Leitura de Sensores e Potenciômetro
    // É importante ler os sensores frequentemente para ter os dados mais atualizados.
    currentIncubatorTemp = readTemperature(incubatorTempPin, incubatorTermistor);
    currentHeatSinkTemp = readTemperature(heatSinkTempPin, heatSinkTermistor);
    targetTemp = map(analogRead(potPin), 0, 1023, 36, 38); // Mapeia para faixa de chocadeira (36 a 38 C)
    MAX_HEAT_SINK_TEMP = targetTemp;
    // 2. Verificação de Segurança da Temperatura do Dissipador de Calor (Não-Bloqueante)
    // Esta verificação é crucial para proteger a pastilha Peltier de superaquecimento do dissipador.
    if (currentTime - lastHeatSinkCheckTime >= HEAT_SINK_CHECK_INTERVAL) {
        lastHeatSinkCheckTime = currentTime; // Atualiza o tempo da última verificação

        if (currentHeatSinkTemp >= MAX_HEAT_SINK_TEMP && !peltier_disabled_by_safety) {
            // Se a temperatura do dissipador exceder o limite E a Peltier ainda não estiver desabilitada,
            // desliga a Peltier e ativa a flag de segurança.
            Serial.println("ATENCAO: Dissipador Quente EXCEDIDO! Peltier DESLIGADA.");
            Timer1.pwm(peltierPin, 0); // Desliga a Peltier imediatamente
            peltier_disabled_by_safety = true; // Define a flag para desabilitar o controle PID
        } else if (currentHeatSinkTemp < (MAX_HEAT_SINK_TEMP - HEAT_SINK_HYSTERESIS) && peltier_disabled_by_safety) {
            // Se a temperatura do dissipador cair abaixo do limite com histerese E a Peltier estiver desabilitada,
            // reativa o controle PID.
            Serial.println("Temperatura do dissipador normalizada. Reativando Peltier.");
            peltier_disabled_by_safety = false; // Reativa o controle PID da Peltier
        }
    }

    // 3. Cálculo do PID (Executado a cada PID_INTERVAL)
    // Este bloco só é executado quando o intervalo de tempo definido para o PID é atingido.
    if (currentTime - lastProcessTime >= PID_INTERVAL) {
        lastProcessTime = currentTime; // Atualiza o tempo para a próxima iteração do PID

        // Se a Peltier estiver desabilitada devido à alta temperatura do dissipador,
        // força a saída do PID para 0 e reseta o termo integral para evitar "wind-up".
        if (peltier_disabled_by_safety) {
            PID_output = 0.0;
            I = 0.0; // Reseta o termo integral para evitar acúmulo enquanto a Peltier está desabilitada
        } else {
            // Calcula o erro: (Temperatura Alvo - Temperatura Atual da Chocadeira)
            // Para aquecimento: erro positivo significa que a temperatura está baixa e precisa aquecer.
            error = targetTemp - currentIncubatorTemp;

            if (error <= 0) { // Se a temperatura estiver no alvo ou acima, desliga o aquecimento
                PID_output = 0.0;
                I = 0.0; // Reseta o termo integral quando o alvo é atingido ou excedido
            } else {
                // Cálculo dos termos P, I e D
                P = error * Kp;
                I = I + error * Ki; // Termo integral: acumula o erro ao longo do tempo

                // Limitação do termo integral para evitar "wind-up" (saturação do integrador)
                if (I > 500) I = 500; // Limite superior para I
                if (I < 0) I = 0;     // I não deve ser negativo para aquecimento apenas

                // Termo derivativo: reage à taxa de mudança da temperatura
                // (currentIncubatorTemp - lastIncubatorTemp) indica se a temperatura está subindo ou descendo
                // Se a temperatura está subindo (currentIncubatorTemp > lastIncubatorTemp), D será positivo,
                // o que reduzirá a saída do PID (freio).
                // Se a temperatura está caindo (currentIncubatorTemp < lastIncubatorTemp), D será negativo,
                // o que aumentará a saída do PID (acelerador).
                D = ((currentIncubatorTemp - lastIncubatorTemp) * Kd);
                
                // Saída final do PID
                // Para aquecimento, D é subtraído porque um aumento de temperatura deve reduzir o PID (freio).
                PID_output = P + I - D; 
            }
        }

        lastIncubatorTemp = currentIncubatorTemp; // Atualiza a última temperatura para o próximo cálculo de D

        // 4. Limitação da Saída do PID para a Faixa do PWM (0-1023)
        // Garante que o valor de saída do PID esteja dentro dos limites aceitáveis para o PWM.
        if (PID_output < 0) {
            // Se o PID_output for negativo (indicaria resfriamento ou nenhum aquecimento),
            // ele é limitado a 0, pois a Peltier está configurada apenas para aquecer.
            PID_output = 0.0;
        } else if (PID_output > 1023) {
            // Se o PID_output exceder o máximo do PWM, ele é limitado a 1023.
            PID_output = 1023.0;
        }
        
        // 5. Mudar o pulso do PWM da Peltier
        // Converte a saída do PID para um inteiro e aplica ao pino da Peltier.
        Timer1.pwm(peltierPin, int(PID_output));

        // 6. Imprime os valores para depuração no Serial Monitor e atualiza o LCD
        printDebugInfo();
        updateLCD();
    }
}

/**
 * @brief Calcula a temperatura em Celsius a partir da leitura analógica de um termistor.
 * @param pin O pino analógico onde o termistor está conectado.
 * @param termistorInstance A instância da biblioteca Aci_10K_an para o termistor específico.
 * @return A temperatura em graus Celsius.
 */
float readTemperature(int pin, Aci_10K& termistorInstance) {
    // Esta função usa a biblioteca ACI_10K_an para termistores NTC 10K,
    // que lida com a não-linearidade do sensor para fornecer uma leitura precisa.
    // Certifique-se de que a resistência de pull-up para o termistor está correta (geralmente 10k Ohms).
    return termistorInstance.getTemp(analogRead(pin));
}

/**
 * @brief Imprime informações de depuração no Serial Monitor.
 */
void printDebugInfo() {
    Serial.println("#########################################");
    Serial.print("Temp Chocadeira: ");
    Serial.print(currentIncubatorTemp);
    Serial.println(" C");

    Serial.print("Temp Dissipador: ");
    Serial.print(currentHeatSinkTemp);
    Serial.println(" C");

    Serial.print("Temp Alvo: ");
    Serial.print(targetTemp);
    Serial.println(" C");

    Serial.print("Erro: ");
    Serial.println(error);

    Serial.print("PID Output (PWM): ");
    Serial.println(int(PID_output));
    Serial.print("Peltier Desabilitada por Seguranca: ");
    Serial.println(peltier_disabled_by_safety ? "SIM" : "NAO");
}

/**
 * @brief Atualiza o display LCD com as informações do sistema.
 */
void updateLCD() {
    lcd.setCursor(0, 0); // Define o cursor para a primeira coluna, primeira linha
    lcd.print("TCh:");
    lcd.print(currentIncubatorTemp, 1); // Exibe a temperatura da chocadeira com 1 casa decimal
    lcd.print(" TP:");
    lcd.print(targetTemp, 1); // Exibe a temperatura alvo com 1 casa decimal

    lcd.setCursor(0, 1); // Define o cursor para a primeira coluna, segunda linha
    lcd.print("TDf:");
    lcd.print(currentHeatSinkTemp, 1); // Exibe a temperatura do dissipador com 1 casa decimal
    lcd.print(" PWM:");
    lcd.print(int(PID_output)); // Exibe a saída do PID (PWM) como um inteiro
}
