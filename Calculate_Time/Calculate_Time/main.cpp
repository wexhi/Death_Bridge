#include<iostream>
#include<iomanip>
using namespace std;

#define PI 3.141592653
#define CAR_R 0.0220

int main() {
	//const double v = 1.53;//米每秒
	const double d = 0.17;//两轮之间的距离 
	double Len[20] = {
	0,  0.0791  ,  0.0815  ,  0.0859  ,  0.0921  ,  0.0995  ,  0.1076  ,  0.1162  ,  0.1249  ,  0.1336  ,  0.1419,
		0.1497  ,  0.1570  ,  0.1636  ,  0.1694  ,  0.1743  ,  0.1783  ,  0.1813  ,  0.1834  ,  0.1844
	};
	double theta[20] = {
	0,  0.4354  ,  0.4204  ,  0.4004  ,  0.3795  ,  0.3605  ,  0.3442  ,  0.3308  ,  0.3201  ,  0.3114  ,  0.3046,
		0.2991  ,  0.2948  ,  0.2914  ,  0.2887  ,  0.2867  ,  0.2852  ,  0.2842  ,  0.2836  ,     0
	};
	double R[19] = {
	0,  0.1832  ,  0.1953 ,   0.2161 ,   0.2441 ,   0.2774  ,  0.3142  ,  0.3529  ,  0.3921  ,  0.4306  ,  0.4676,
		0.5025  ,  0.5345  ,  0.5633  ,  0.5886  ,  0.6100  ,  0.6273  ,  0.6403  ,  0.6489
	};
	double Lv[20], Rv[20];//左轮pwm与右轮pwm 
	double t[20];//每一段的时间 

	//计算速度
	double n, v = 0;
	cin >> n;
	v = 2 * PI * CAR_R * n;
	
	//计算时间 
	for (int i = 1; i <= 19; i++)t[i] = Len[i] / v;
	//计算pwm 
	for (int i = 1; i <= 18; i++) {
		Lv[i] = v - d * (v / R[i]);
		Rv[i] = v;
		//Lv[i] = ceil(Lv[i] / v * 7199);
		//Rv[i] = ceil(Rv[i] / v * 7199);
		//将线速度转化为转速
		Lv[i] = Lv[i] / (2 * PI * CAR_R);
		Rv[i] = Rv[i] / (2 * PI * CAR_R);

	}
	//输出数据 
	for (int i = 1; i <= 18; i++)//printf("%.0f ", Lv[i]);
	{
		cout<<fixed<<setprecision(3)<<Lv[i]<<", ";
	}
	cout << endl;
	for (int i = 1; i <= 18; i++)//printf("%.0f ", Rv[i])
	{
		cout << fixed << setprecision(3) << Rv[i] << ", ";
	}
	cout << endl;
	for (int i = 1; i <= 18; i++)//printf("%.0f ", t[i] * 1000);//毫秒 
	{
		cout << fixed << setprecision(3) << t[i] * 1000 << ", ";
	}
}
