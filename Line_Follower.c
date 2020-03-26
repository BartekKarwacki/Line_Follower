#include<avr/io.h>
#include<avr/interrupt.h>
#define BIN1 PD6 
#define BIN2 PD7
#define AIN1 PD2
#define AIN2 PD4
#define PWMA PD3
#define PWMB PD5
#define STBY PB4

void ADC_init()     
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADEN);
	ADMUX |= (1 << ADLAR) | (1 << REFS0);
}
void GPIO_TIM_init()
{
	// Konfiguracja wyjœæ silnika
	DDRD |= (1 << PWMA);  //PWM Silnika A na wyjœcie
	DDRD |= (1 << PWMB); // PWM Silnika B na wyjœcie
	DDRD |= (1 << BIN1); // Wyjscie nr 1 na mostek silnika B 
	DDRD |= (1 << BIN2); // Wyjscie nr 2 na mostek silnika B 
	DDRB |= (1 << AIN1); // Wyjscie nr 1 na mostek silnika A 
	DDRD |= (1 << AIN2); // Wyjscie nr 2 na mostek silnika A 
	DDRB |= (1 << STBY); // Ustawienie STBY mostka na wyjœcie

	// Konfiguracja Timera 2 PWM
	TCCR2A |= (1 << WGM20)|(1<<WGM21); // Ustawienie trybu FAST PWM
	TCCR2B |= (1 << CS20);// Wybor prescalera 1 (No prescaling)
	TCCR2A |= (1 << COM2B1); // Zerowanie pinu przy zliczaniu w dó³, ustawianie przy zliczaniu w górê

	//Konfiguracja Timera 0 PWM 
	TCCR0A |= (1 << WGM00)|(1<<WGM01);// Ustawienie trybu FAST PWM
	TCCR0B |= (1 << CS00); // Wybor prescalera 1 (No prescaling)
	TCCR0A |= (1 << COM0B1);  // Zerowanie pinu przy zliczaniu w dó³, ustawianie przy zliczaniu w górê

	// Timer 1 do cyklicznego wykonywania petli 
	TCCR1B |= (1 << WGM12); //Ustawienie trybu CTC
	TCCR1B |= (1 << CS10) | (1 << CS12); // Wybór prescalera 1024
	OCR1A = 156;// Czestotliwosc przerwania 100Hz - (16*10^6)/1024/156
	TIMSK1 |= (1 << OCIE1A); // zezwolenie na przerwanie CompareMatch
	
	//Ustawienie pocz¹tkowego wypelnienia PWM
	OCR2B = 0;  
	OCR0B = 0;

	// Ustawienie "1" na mostek dla silników i STBY
	PORTD |= (1 << AIN2);
	PORTD |= (1 << BIN2);
	PORTB |= (1 << STBY);
}
//Zmienne globalne
uint8_t tab_czujnikow[8] = { 4, 7, 6, 5, 0, 1, 3, 2 };// TABLICA  Z ZAPISANYMI NUMERAMI PINÓW  CZUJNIKÓW
int8_t czujniki[8] = { 0 };  //WARTOSCI ODCZYTANE Z CZUJNIKA
uint8_t V_zal = 130;  // Za³o¿one wype³nienie PWM(OD 0 DO 255)
int8_t waga[8] = { 8,5,4,1, -1, -4, -5, -8 }; //WAGA CZUJNIKÓW UMO¯LIWIA ZRÓ¯NICOWANIE GWA£TOWNOŒCI REAKCJI
int8_t error_last = 0; //POPRZEDNI B£¥D POOZYCJA
int8_t error = 0;  //B£¥D POZYCJI
float uchyb, ostatni_uchyb = 0, d_blad; // zmienne do regulatora pd
uint8_t Kp = 15; // czlon proporcjonalny 
uint8_t Kd = 65; // czlon rozniczkujacy 


void ADC_read()    //funkcja sprawdzajaca, czy czujnik znajduje sie nad linia
{
	for (uint8_t i = 0; i < 8; i++)
	{
		ADMUX &= 0b11100000; // 2 najstarsze bity - wewnetrzne odniesienie 1.1V
		ADMUX |= tab_czujnikow[i]; // Ustawienie, z którego wejscia ma zczytywac wartosci
		ADCSRA |= (1 << ADSC); // Wybranie pojedyñczej konwersji 
		while (ADCSRA &(1<<ADSC)) {}; // Petla dziala dopóki nie zostanie odczytana wartoœæ, gdy zostanie odczytana to bit ADSC zmienia siê na 0
		 // Wynik znajduje siê w tym rejestrze -When an ADC conversion is complete, the result is found in these two registers. 
		if (ADCH > 150)  // Je¿eli ADCH jest wiêksze ni¿ 150 to czujnik znajduje siê pod linia 
		{
			czujniki[i] = 1;
		}
		else
		{
			czujniki[i] = 0;
		}

	}

}

float Blad()// FUNKCJANA ZWRACA B£¥D POZYCJI
{
	error = 0; // zerowanie erroru - zmienna globalna 
	uint8_t lczujnikow = 0; // ile czujników jest aktywnych jednoczesnie
	for (uint8_t i = 0; i < 8; i++)
	{
		error = error + czujniki[i] * waga[i]; // error to suma wag, które widzialy czujniki
		lczujnikow += czujniki[i]; 

	}

	if (lczujnikow != 0)
	{
		error /= lczujnikow; // blad wynosi suma wag przez ilosc czujnikow
		error_last = error; // ustaw jako ostatni error
	}
	else
	{
		error = error_last; // ustaw error jako poprzedni error
	}
	return error;

}

int16_t PD() // funckja realizujaca algorytm PD
{

	d_blad = uchyb - ostatni_uchyb;

	ostatni_uchyb = uchyb;
	return Kp * uchyb + Kd * d_blad;
}

void PWM(int16_t lewy, int16_t prawy)// funkcja ustawiajaca wyjscia pwm argumenty to wypelnienie poczatkowe i regulacja
{
	if (lewy >= 0) {
		if (lewy > 255)
			lewy = 255;
		// Ustawienie odpowiednich kierunkow obrotow silnika
		PORTD |= (1 << AIN2); 
		PORTD &= ~(1 << AIN1);

	}
	else
	{
		if (lewy < -255)
			lewy = -255;
		// Ustawienie odpowiednich kierunkow obrotow silnika
		PORTD |= (1 << AIN1);
		PORTD &= ~(1 << AIN2);


	}

	if (prawy >= 0)
	{
		if (prawy > 255)
			prawy = 255;
		// Ustawienie odpowiednich kierunkow obrotow silnika
		PORTD |= (1 << BIN2);
		PORTD &= ~(1 << BIN1);

	}
	else
	{
		if (prawy < -255)
			prawy = -255;
		// Ustawienie odpowiednich kierunkow obrotow silnika
		PORTD |= (1 << BIN1);
		PORTD &= ~(1 << BIN2);
	}
	// ustawienie wypelnienia 
	OCR2B = abs(lewy);
	OCR0B = abs(prawy);

}

ISR(TIMER1_COMPA_vect) // Przerwanie glowne (glowna petla)
{
	ADC_read(); // zczytaj czujniki 
	uchyb = Blad(); // uchyb jest rowny errorowi
	int16_t regulacja = PD(); // regulacja wynikiem PD
	PWM(V_zal - regulacja, V_zal + regulacja); // ustawienie wypelnienia - poczatkowe i regulacja
}
int main(void)
{
	GPIO_TIM_init();
	ADC_init();
	sei(); // zezwolenie na przerwania
	while (1)
	{
	}
}

