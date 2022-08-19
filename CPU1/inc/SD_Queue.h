/*
 * SD_Queue.h
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#ifndef INC_SD_QUEUE_H_
#define INC_SD_QUEUE_H_
//16bit = 1byte ����
//������ �����͸����� Ư���� �°� �����Ѵ�.(��ǥ������ Update �ֱ�)
//Queue ������� Packet�� ũ��� �ֱ⿡ �°� �����Ѵ�....���� Hz�ϼ��� Queue ����� �ø���
//�̷��� �Ͽ� �� ��������� �޸𸮰������� ���� Heap size�� �Ѿ�� �ȵȴ�!!
//100Hz Sensor data(IMU..ADIS16448)...ID = 0x05
#define REC1_SZ   17 // [ HEADER(3) + ID(1) + Size(2)+ MCUTIME(4) + GYRO(6) + ACC(6) + MAG(6) + Baro(2) + Temp(2) + Checksum(2) ]/ 2 ..�������� ������ �ݿø��Ѵ�
#define QUE1_SZ   500

#if defined(NOVATEL_OEM_719)
//10Hz Sensor data(Novatel BESTXYZ)...ID = 0x0A
#define REC2_SZ   68 // [ HEADER(3) + ID(1) + Size(2) + BESTXYZPACK(130)]/ 2
#define QUE2_SZ   20

//Changeable
//10Hz Sensor data(Novatel RANGE)...ID = 0x0B.. This packet size is changeable.
#define REC3_SZ   892
#define QUE3_SZ   10

#elif defined(UBLOX_NEO_8)
//5Hz Sensor data(Ublox PVT)...ID = 0x06
#define REC2_SZ   42 // [ HEADER(3) + ID(1) + Size(2) + PVTPACK(78)]/ 2
#define QUE2_SZ   40

//Changeable
//5Hz Sensor data(Ublox SAT)...ID = 0x07 This packet is changeable.
#define REC3_SZ   430 // [ HEADER(3) + ID(1) + Size(2) + MCUTIME(4) + CHECKSUMFLAG(2)+
                      // SATPACK(Always change and set the Maximum Satellite 55.. So Maximum size is 8+12*70 = 848)]/ 2
#define QUE3_SZ   40

#endif

//100Hz Navigation Solution and Control result[���� ����]
//#define REC3_SZ   200
//#define QUE3_SZ   15


typedef struct {

    int16* Q_BUFFER;

} structQueueArray;

typedef struct {

    Uint16 R_SIZE;
    Uint16 Q_SIZE;
    structQueueArray* pstQarr;
    Uint16 q_head;
    Uint16 q_tail;
} structLogQueue;

void Log_Init(void);
void Log_EnQueue(int16* pPacket);
void QueueWrite(void);
inline void increase_Queue_value(Uint16 * data_p);
void Log_EnQueue2(int16* pPacket);
void Queue2Write(void);
inline void increase_Queue2_value(Uint16 * data_p);
void Log_EnQueue3(int16* pPacket);
void Queue3Write(void);
inline void increase_Queue3_value(Uint16 * data_p);


#endif /* INC_SD_QUEUE_H_ */
