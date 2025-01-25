//Importando as bibliotecas:
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include "math.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "soc/clk_tree_defs.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"



//Definição dos pinos utilizados:
#define E1 0					//Encoder 1 (ADC channel)
#define E2 1					//Encoder 2 (ADC channel)
#define E3 4					//Encoder 3 (ADC channel)
#define E4 5					//Encoder 4 (ADC channel)
#define E5 6					//Encoder 5 (ADC channel)
#define E6 7					//Encoder 6 (ADC channel)

const int PM1[2] = {11, 12};	//Motor 1
const int PM2[2] = {14, 13};	//Motor 2
const int PM3[2] = {15, 16};	//Motor 3
const int PM4[2] = {18, 17};	//Motor 4
const int PM5[2] = {39, 40};	//Motor 5
const int PM6[2] = {41, 42};	//Motor 6

#define ISS1 21					//IS Select 1
#define ISS2 35					//IS Select 2
#define ISS3 36					//IS Select 3
#define ISS4 37					//IS Select 4
#define ISS5 38					//IS Select 5
#define ISS6 47					//IS Select 6

#define ISAR1 8					//IS Analog Read 1 (ADC channel)
#define ISAR2 9					//IS Analog Read 2 (ADC channel)



//Definição de constantes:
#define ts 10				// tempo de amostragem, em milissegundos (amostragem e controlador)
#define tt 100				// step de tempo para a geração de trajetória
#define SPD_ANG 10			// velocidade angular (graus por segundo)
#define SPD_LIN 100		// velocidade linear (mm/s)
#define SKIP_SAMPLES 10	// frequência de envio para a IHM (a cada 5 amostras reais, 1 é enviada)
#define BUFF_LEN 1			// tamanho do buffer de envio dos dados
#define kf 50				// constante do filtro digital de amostragem
#define L 5					// deslocamento do arranjo diferencial
#define USAT 50				// saturação do sinal de controle

//Constantes padrão para o controlador PID para cada grau de liberdade:
const float DEFAULT_PID[6][3] = {
		{0.8, 0, 0},
		{0.8, 0, 0},
		{1.0, 0, 0},
		{1.0, 0, 0},
		{1.0, 0, 0},
		{1.0, 0, 0},
};

//Constantes atuais controlador PID para cada grau de liberdade:
float KPID[6][3];

//Constantes de calibração das juntas (ajuste linear: primeiro trecho/segundo trecho):
const float CAL[6][2][2] = {
		{{-0.1917, 334.6199}, {-0.1917, 334.6199}},
		{{-0.1861, 186.8597}, {-0.1861, 186.8597}},
		{{-0.1956, 204.9627}, {-0.1956, 204.9627}},
		{{-0.1873, 373.9334}, {-0.1873, 373.9334}},
		{{-0.1983, 76.8438}, {-0.1871, 419.7772}},
		{{-0.0537, 66.2583}, {-0.0464, 150.7178}},
};
const int THRESHOLDS[6] = {4000, 4000, 4000, 4000, 1250, 1250}; //A partir de qual valor, em bits, usar o segundo trecho de CAL

//Zonas mortas dos motores (setnido negativo/positivo):
float ZM[6][3] = {
		{2.5, 0, 2.5},
		{2.5, 0, 2.5},
		{2.0, 0, 2.0},
		{2.5, 0, 2.5},
		{2.5, 0, 2.5},
		{3.5, 0, 3.5},
};

//Constantes de trajetória:
float trajconsts[5][4];



//Struct para dados do motor:
typedef struct
{
    int motor_pin_1;
    int motor_pin_2;
    ledc_channel_t pwm_channel;
} motor_t;

//Definição dos motores:
const motor_t M1 = {
	.motor_pin_1 = PM1[0],
	.motor_pin_2 = PM1[1],
	.pwm_channel = LEDC_CHANNEL_0
};
const motor_t M2 = {
	.motor_pin_1 = PM2[0],
	.motor_pin_2 = PM2[1],
	.pwm_channel = LEDC_CHANNEL_1
};
const motor_t M3 = {
	.motor_pin_1 = PM3[0],
	.motor_pin_2 = PM3[1],
	.pwm_channel = LEDC_CHANNEL_2
};
const motor_t M4 = {
	.motor_pin_1 = PM4[0],
	.motor_pin_2 = PM4[1],
	.pwm_channel = LEDC_CHANNEL_3
};
const motor_t M5 = {
	.motor_pin_1 = PM5[0],
	.motor_pin_2 = PM5[1],
	.pwm_channel = LEDC_CHANNEL_4
};
const motor_t M6 = {
	.motor_pin_1 = PM6[0],
	.motor_pin_2 = PM6[1],
	.pwm_channel = LEDC_CHANNEL_5
};

//Struct de dados para o buffer:
typedef struct
{
    float dof[BUFF_LEN][6];
    float ref[BUFF_LEN][6];
    float u[BUFF_LEN][6];
    bool full;
} Buffer;
Buffer buff[2];

//Vetores de dispositivos:
const int ENCODERS[6] = {E1, E2, E3, E4, E5, E6};
const motor_t MOTORS[6] = {M1, M2, M3, M4, M5, M6};
const int ISS[6] = {ISS1, ISS2, ISS3, ISS4, ISS5, ISS6};



//Declaração e inicialização de variáveis globais:
adc_oneshot_unit_handle_t adc_handle = NULL;
esp_mqtt_client_handle_t client = NULL;
esp_timer_handle_t aftertraj_timer;
float TARGETS[6] = {0, 0, 0, 0, 0, 0};
float REFS[6] = {0, 0, 0, 0, 0, 50};
float DOFS[6] = {0, 0, 0, 0, 0, 0};
float U[6] = {0, 0, 0, 0, 0, 0};
float E[6] = {0, 0, 0, 0, 0, 0};
float Ea[6] = {0, 0, 0, 0, 0, 0};
float Eaa[6] = {0, 0, 0, 0, 0, 0};
float IE[6] = {0, 0, 0, 0, 0, 0};
float DE[6] = {0, 0, 0, 0, 0, 0};
float dofa[6] = {0, 0, 0, 0, 0, 0};
bool JNT_ENABLE[6] = {false, false, false, false, false, false};
char topic[15], data[10];
bool move=false, trajenable=false;
int counter=0, current_buff_id=0, iterations=0, current_iter=0;
float SPD_OVR=50;



//Declaração de funções:
void updatePWM();
void jnt_disable_all();

//Função para retornar o sinal (1 ou -1):
int sign(float num)
{
	return (int)(num/fabs(num));
}

//Função para saturação do sinal:
float sat(float num, int lim_inf, int lim_sup)
{
	if(num > lim_sup)
	{
		return lim_sup;
	}
	if(num < lim_inf)
	{
		return lim_inf;
	}
	return num;
}

//Função para conferir os limites angulares das juntas:
bool checklimits()
{
	float limits[6][2] = {{-170, 170}, {-90, 30}, {-135, 0}, {-120, 120}, {-170, 170}};
	bool valid = true;
	for(int i=0; i<5; i++)
	{
		valid = valid&&(TARGETS[i]>=limits[i][0])&&(TARGETS[i]<=limits[i][1]);
	}
	return valid;
}

//Função para retornar módulo do maior deslocamento angular:
float maxdelta()
{
	float mv = 0;
	for(int i=0; i<5; i++)
	{
		if(fabs(TARGETS[i]-DOFS[i])>mv)
		{
			mv = fabs(TARGETS[i]-DOFS[i]);
		}
	}
	return mv;
}

//Cálculo das constantes para a trajetória:
void trajCalc()
{
	iterations = round((maxdelta()/(float)(SPD_ANG*SPD_OVR*1e-2))/(float)(tt*1e-3));
	for(int i=0; i<5; i++)
	{
		float q0 = DOFS[i];
		trajconsts[i][0] = q0;
		trajconsts[i][1] = (TARGETS[i]-q0)*10/(float)pow(iterations, 3);
		trajconsts[i][2] = (TARGETS[i]-q0)*-15/(float)pow(iterations, 4);
		trajconsts[i][3] = (TARGETS[i]-q0)*6/(float)pow(iterations, 5);
	}
}

//Calcula o valor da trajetória:
float trajVal(int joint, int sample)
{
	return (trajconsts[joint][0]+(trajconsts[joint][1]*pow(sample,3))+(trajconsts[joint][2]*pow(sample,4))+(trajconsts[joint][3]*pow(sample,5)));
}

//Função de callback para geração de tragetória:
static void aftertraj(void* arg)
{
	esp_mqtt_client_publish(client, "reply", "J1", 0, 0, 0);
	jnt_disable_all();
}

//Função de callback para geração de tragetória:
static void trajgen(void* arg)
{
	if(trajenable)
	{
		current_iter++;
		for(int i=0; i<5; i++)
		{
			REFS[i] = trajVal(i, current_iter);
		}
		if(current_iter == iterations)
		{
			trajenable = false;
			current_iter = 0;
			//esp_timer_start_once(aftertraj_timer, 1000e3); //aguarda para desabilitar as o controle das juntas
		}
	}
}

//Função para configurar e iniciar o wifi:
void wifi()
{
	//Inicializando wifi:
	nvs_flash_init();
	esp_netif_init();
	esp_event_loop_create_default();
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&wifi_initiation);
	wifi_config_t wifi_configuration = {
		.sta = {
					.ssid 		= "Erick",
					.password 	= "fanplate",
		},
	};
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
	esp_wifi_start();
	esp_wifi_connect();
}

//Funções para habilitar e desabilitar o controle das juntas:
void jnt_enable_all()
{
	for(int i=0; i<6; i++)
	{
		JNT_ENABLE[i] = true;
	}
}
void jnt_disable_all()
{
	for(int i=0; i<6; i++)
	{
		JNT_ENABLE[i] = false;
		U[i] = 0;
	}
	updatePWM();
}

//Função para resetar os integradores:
void integralReset()
{
	for(int i=0; i<6; i++)
	{
		IE[i] = 0;
	}
}

//Função de callback do MQTT:
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	esp_mqtt_event_handle_t event = event_data;
	sprintf(topic, "%.*s", event->topic_len, event->topic);
	sprintf(data, "%.*s", event->data_len, event->data);
	//printf("%s\t\%s\n", topic, data);
	if(1)//(!strcmp(topic, "request"))
	{
		if(strlen(data)==1)	//Comandos diretos:
		{
			if(data[0] == 'S')
			{
				move = true;
			}
			if(data[0] == 'H')
			{
				move = false;
				jnt_disable_all();
			}
		}else	//Comandos compostos:
		{
			//Segmentação dos parâmetros:
			int i=0, pos=0, mark=0;
			float consts[6];
			char val[10];
			for(i=1; i<event->data_len; i++)
			{
				if(data[i] != ',')
				{
					val[i-mark-1] = data[i];
				}else
				{
					val[i-mark-1] = '\0';
					consts[pos] = atof(val);
					pos++;
					mark = i;
					val[0] = '\0';
				}
			}
			//Identificação e atribuição do comando:
			if(data[0] == 'J')	//move joint
			{
				for(int i=0; i<5; i++)
				{
					TARGETS[i] = consts[i];
				}
				if(checklimits())
				{
					trajCalc();
					current_iter = 0;
					trajenable = true;
					jnt_enable_all();
				}else
				{
					esp_mqtt_client_publish(client, "reply", "J0", 0, 0, 0);
				}
			}
			if(data[0] == 'D')	//velocidade
			{
				SPD_OVR = consts[0];
			}
			if(data[0] == 'O')	//PWM em malha aberta
			{
				U[(int)consts[0]-1] = sat(consts[1], -30, 30);
				updatePWM();
			}
			if(data[0] == 'C')	//Altera as constantes do controlador
			{
				integralReset();
				for(int i=0; i<3; i++)
				{
					KPID[(int)consts[0]-1][i] = consts[i+1];
				}
			}
			if(data[0] == 'G')	//Atribui as referências aos controladores
			{
				JNT_ENABLE[(int)consts[0]-1] = true;
				REFS[(int)consts[0]-1] = consts[1];
			}
		}
	}
}

//Função para configurar e iniciar o serviço MQTT:
void mqtt()
{
	//Conexão com o broker e ID do dispositivo:
	esp_event_loop_create_default();
	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = "mqtt://192.168.137.1:1884/mqtt",
		.credentials.client_id	= "CONTROLLER",
	};
	client = esp_mqtt_client_init(&mqtt_cfg);

	//Configurando a task que é acionada sempre que receber mensagem:
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

	//Iniciando o serviço MQTT:
	esp_mqtt_client_start(client);

	//Subscrevendo nos tópicos:
	esp_mqtt_client_subscribe(client, "request", 0);
}

//Função para configurar o PWM:
void pwm()
{
	//Configuração do timer para geração do PWM:
	ledc_timer_config_t pwmc = {
		.speed_mode 		= LEDC_LOW_SPEED_MODE,
		.timer_num 			= LEDC_TIMER_0,
		.duty_resolution 	= LEDC_TIMER_12_BIT,
		.freq_hz 			= 10e3,
		.clk_cfg 			= LEDC_AUTO_CLK
	};
	ledc_timer_config(&pwmc);
}

//Função para enviar o PWM aos motores:
void set_PWM(motor_t motor, float sig)
{
	gpio_set_direction(motor.motor_pin_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(motor.motor_pin_2, GPIO_MODE_OUTPUT);
	if(sig>=0)
	{
		gpio_set_level(motor.motor_pin_1, 0);
		ledc_channel_config_t pwmch = {
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.channel 	= motor.pwm_channel,
			.timer_sel 	= LEDC_TIMER_0,
			.intr_type 	= LEDC_INTR_DISABLE,
			.gpio_num 	= motor.motor_pin_2,
			.duty		= abs((int)(sig*4095.0/100.0)),
			.hpoint		= 0,
		};
		ledc_channel_config(&pwmch);
	}else
	{
		gpio_set_level(motor.motor_pin_2, 0);
		ledc_channel_config_t pwmch = {
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.channel 	= motor.pwm_channel,
			.timer_sel 	= LEDC_TIMER_0,
			.intr_type 	= LEDC_INTR_DISABLE,
			.gpio_num 	= motor.motor_pin_1,
			.duty		= abs((int)(sig*4095.0/100.0)),
			.hpoint		= 0,
		};
		ledc_channel_config(&pwmch);
	}
}

//Função para configurar o ADC:
void ADC()
{
	adc_oneshot_unit_init_cfg_t init_cfg = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	adc_oneshot_new_unit(&init_cfg, &adc_handle);

	adc_oneshot_chan_cfg_t ch_cfg = {
		.bitwidth = ADC_BITWIDTH_12,
		.atten = ADC_ATTEN_DB_11,
	};
	adc_oneshot_config_channel(adc_handle, E1, &ch_cfg);
	adc_oneshot_config_channel(adc_handle, E2, &ch_cfg);
	adc_oneshot_config_channel(adc_handle, E3, &ch_cfg);
	adc_oneshot_config_channel(adc_handle, E4, &ch_cfg);
	adc_oneshot_config_channel(adc_handle, E5, &ch_cfg);
	adc_oneshot_config_channel(adc_handle, E6, &ch_cfg);
	adc_oneshot_config_channel(adc_handle, ISAR1, &ch_cfg);
	adc_oneshot_config_channel(adc_handle, ISAR2, &ch_cfg);
}

//Função de calibração das juntas:
float calib(int bitval, int dof)
{
	if(bitval<THRESHOLDS[dof])
	{
		return ((CAL[dof][0][0]*bitval)+CAL[dof][0][1]);
	}else
	{
		return ((CAL[dof][1][0]*bitval)+CAL[dof][1][1]);
	}
}

//Função para atualizar o valor das juntas:
void updateDOFS()
{
	for(int i=0; i<6; i++)
	{
		int val;
		adc_oneshot_read(adc_handle, ENCODERS[i], &val);
		DOFS[i] += (calib(val, i)-DOFS[i])/(float)kf;
	}
}

//Função para cálculo do PID:
void PID()
{
	float u;
	//Para as três primeiras juntas:
	for(int i=0; i<3; i++)
	{
		if(JNT_ENABLE[i])
		{
			//Eaa[i] = Ea[i];
			//Ea[i] = E[i];
			E[i] = REFS[i]-DOFS[i];
			IE[i] += (E[i]*ts*1e-3);
			DE[i] = (DOFS[i]-dofa[i])/(float)(ts*1e-3);
			dofa[i] = DOFS[i];
			u = ((KPID[i][0]*E[i]) + (KPID[i][1]*IE[i]) + (KPID[i][2]*DE[i]));
			//u = ((KPID[i][1]*ts*1e-3/2.0)+(KPID[i][2]/(float)(ts*1e-3))+KPID[i][0])*E[i];
			//u += ((KPID[i][1]*ts*1e-3/2.0)-(2*KPID[i][2]/(float)(ts*1e-3))-KPID[i][0])*Ea[i];
			//u += (KPID[i][2]/(float)(ts*1e-3))*Eaa[i];
			u += sign(u)*ZM[i][sign(u)+1];

			U[i] = sat(u, -USAT, USAT);
		}
	}
	//Para o arranjo diferencial:
	if(JNT_ENABLE[3])
	{
		float refth4_ = (REFS[3]+(REFS[4]*L/2.0));
		float th4_ = (DOFS[3]+(DOFS[4]*L/2.0));
		E[3] = refth4_-th4_;
		IE[3] += (E[3]*ts*1e-3);
		DE[3] = (DOFS[3]-dofa[3])/(float)(ts*1e-3);
		dofa[3] = DOFS[3];
		u = ((KPID[3][0]*E[3]) + (KPID[3][1]*IE[3]) + (KPID[3][2]*DE[3]));
		u += sign(u)*ZM[3][sign(u)+1];
		U[3] = sat(u, -USAT, USAT);
	}
	if(JNT_ENABLE[4])
	{
		float refth5_ = (REFS[3]-(REFS[4]*L/2.0));
		float th5_ = (DOFS[3]-(DOFS[4]*L/2.0));
		E[4] = refth5_-th5_;
		IE[4] += (E[4]*ts*1e-3);
		DE[4] = (DOFS[4]-dofa[4])/(float)(ts*1e-3);
		dofa[4] = DOFS[4];
		u = ((KPID[4][0]*E[4]) + (KPID[4][1]*IE[4]) + (KPID[4][2]*DE[4]));
		u += sign(u)*ZM[4][sign(u)+1];
		U[4] = sat(u, -USAT, USAT);
	}
	//Para a garra:
	if(JNT_ENABLE[5])
	{
		E[5] = REFS[5]-DOFS[5];
		IE[5] += (E[5]*ts*1e-3);
		DE[5] = (DOFS[5]-dofa[5])/(float)(ts*1e-3);
		dofa[5] = DOFS[5];
		u = ((KPID[5][0]*E[5]) + (KPID[5][1]*IE[5]) + (KPID[5][2]*DE[5]));
		u += sign(u)*ZM[5][sign(u)+1];
		U[5] = sat(u, -USAT, USAT);
	}
}

//Função para aplicar o PWM aos motores:
void updatePWM()
{
	for(int i=0; i<6; i++)
	{
		set_PWM(MOTORS[i], U[i]);
	}
}

//Função para atualizar o buffer:
void att_buff()
{
	if(counter == SKIP_SAMPLES)
	{
		if(current_buff_id<BUFF_LEN)
		{
			for(int i=0; i<6; i++)
			{
				buff[0].dof[current_buff_id][i] = DOFS[i];
				buff[0].ref[current_buff_id][i] = REFS[i];
				buff[0].u[current_buff_id][i] = U[i];
			}
			current_buff_id++;
			if(current_buff_id == BUFF_LEN)
			{
				buff[0].full = true;
			}
		}else
		{
			for(int i=0; i<6; i++)
			{
				buff[1].dof[current_buff_id%BUFF_LEN][i] = DOFS[i];
				buff[1].ref[current_buff_id%BUFF_LEN][i] = REFS[i];
				buff[1].u[current_buff_id%BUFF_LEN][i] = U[i];
			}
			current_buff_id++;
			if(current_buff_id == (2*BUFF_LEN))
			{
				buff[0].full = true;
				current_buff_id = 0;
			}
		}
	}
	counter++;
	if(counter>SKIP_SAMPLES)
	{
		counter = 0;
	}
}

//Função de callback para o timer de controle dedicado:
static void control(void* arg)
{
	//Leitura das juntas:
	updateDOFS();

	if(move)
	{
		PID();
		updatePWM();
	}

	//Atualiza o buffer de envio:
	att_buff();
}

//Função para envio de buffer via MQTT:
void send_data(Buffer buffer)
{
	char aux[10], send1[10*6*BUFF_LEN], send2[10*6*BUFF_LEN], send3[10*6*BUFF_LEN];
	send1[0] = '\0';
	send2[0] = '\0';
	send3[0] = '\0';
	for(int i=0; i<6; i++)
	{
		aux[0] = '\0';
		sprintf(aux, "%.1f", buffer.dof[0][i]);
		strcat(send1, aux);
		///////////////////////////////////////////
		aux[0] = '\0';
		sprintf(aux, "%.1f", buffer.ref[0][i]);
		strcat(send2, aux);
		///////////////////////////////////////////
		aux[0] = '\0';
		sprintf(aux, "%.1f", buffer.u[0][i]);
		strcat(send3, aux);
		if(i<5)
		{
			strcat(send1, ",");
			strcat(send2, ",");
			strcat(send3, ",");
		}
	}
	esp_mqtt_client_publish(client, "sampling1", send1, 0, 0, 0);
	esp_mqtt_client_publish(client, "sampling2", send2, 0, 0, 0);
	esp_mqtt_client_publish(client, "sampling3", send3, 0, 0, 0);
	//aux[0] = '\0';
	//sprintf(aux, "%ld", esp_log_timestamp());
	//esp_mqtt_client_publish(client, "TIME", aux, 0, 0, 0);
}

//Função para looping de envio de dados via MQTT:
void sending_loop(void *args)
{
	Buffer *bf = args;
	//Loop de envio dos dados ao front-end:
	while(1)
	{
		if(bf[0].full)
		{
			send_data(bf[0]);
			bf[0].full = false;
		}
		if(bf[1].full)
		{
			send_data(bf[1]);
			bf[1].full = false;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

//Função para configurar o timer dedicado:
void timers()
{
	//timer peridódico para controle:
	esp_timer_handle_t control_timer;
	const esp_timer_create_args_t control_timer_args = {
		.callback = &control,
	};
	esp_timer_create(&control_timer_args, &control_timer);
	esp_timer_start_periodic(control_timer, (ts*1e3));

	//timer peridódico para geração de trajetória:
	esp_timer_handle_t traj_timer;
	const esp_timer_create_args_t traj_timer_args = {
		.callback = &trajgen,
	};
	esp_timer_create(&traj_timer_args, &traj_timer);
	esp_timer_start_periodic(traj_timer, (tt*1e3));

	//Timer one-shot para desabilitar o controlador após trajetória:
	const esp_timer_create_args_t one_shot_timer_args = {
		.callback = &aftertraj,
	};
	esp_timer_create(&one_shot_timer_args, &aftertraj_timer);

	//Task para envio dos dados:
	xTaskCreate(
			&sending_loop,
			"sending_loop",
			8192,
			&buff,
			1,
			NULL
	);
}

//Função para configurar a GPIO:
void confg_GPIO(int PIN)
{
	gpio_config_t gpioc =
	{
		.pin_bit_mask	= (1ULL << PIN),
		.mode			= GPIO_MODE_OUTPUT,
		.pull_up_en		= GPIO_PULLUP_DISABLE,
		.pull_down_en 	= GPIO_PULLDOWN_DISABLE,
		.intr_type 		= GPIO_INTR_DISABLE,
	};
	gpio_config(&gpioc);
}



void app_main()
{
	//Configuração e inicialização so wifi:
	sleep(1);
	wifi();
	sleep(2);

	//Configuração e inicialização do serviço MQTT:
	mqtt();
	sleep(1);

	//Configuração do PWM:
	pwm();

	//Configuração das GPIO's:
	confg_GPIO(ISS1);
	confg_GPIO(ISS2);
	confg_GPIO(ISS3);
	confg_GPIO(ISS4);
	confg_GPIO(ISS5);
	confg_GPIO(ISS6);

	//Configuração do conversor AD:
	ADC();

	//Inicializações:
	buff[0].full = false;
	buff[1].full = false;
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<3; j++)
		{
			KPID[i][j] = DEFAULT_PID[i][j];
		}
	}

	//Configuração e inicialização dos timers dedicados:
	timers();

	//Aguarda filtros acomodarem e atualiza a configuração articular inicial do robô:
	sleep(1);
	for(int i=0; i<6; i++)
	{
		REFS[i] = DOFS[i];
	}

	//Serial plot:
	/*while(1)
	{
		//int id = 0;
		//printf("\nREF:%g,DOF:%g,U:%g", REFS[id], DOFS[id], U[id]);
		//printf("\t%g\t%g\t%g\n", KPID[0][0], KPID[0][1], KPID[0][2]);
		vTaskDelay(pdMS_TO_TICKS(10));
	}*/
}
