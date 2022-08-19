/*
 * SD_Queue.c
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#include "inc/Global_def.h"

structLogQueue      stLogQueue;  //IMUdata Queue struct
structLogQueue*     pstLogQueue; //IMUdata Queue struct pointer

structLogQueue      stLogQueue2;  //
structLogQueue*     pstLogQueue2; //

structLogQueue      stLogQueue3;  //
structLogQueue*     pstLogQueue3; //
extern FlagStatus   Q_data_write;

void Log_Init(void){
    Uint16 j = 0;
    Uint16 Rec1SZ = {REC1_SZ};
    Uint16 Que1SZ = {QUE1_SZ};
    Uint16 Rec2SZ = {REC2_SZ};
    Uint16 Que2SZ = {QUE2_SZ};
    Uint16 Rec3SZ = {REC3_SZ};
    Uint16 Que3SZ = {QUE3_SZ};

        pstLogQueue = &stLogQueue;
        memset(&stLogQueue, 0x0, sizeof(stLogQueue));
        stLogQueue.R_SIZE = Rec1SZ;
        stLogQueue.Q_SIZE = Que1SZ;

        pstLogQueue->pstQarr = (structQueueArray*)malloc(sizeof(structQueueArray) * Que1SZ);
        for(j = 0; j < Que1SZ; j++)
            pstLogQueue->pstQarr[j].Q_BUFFER = (int16*)malloc(sizeof(int16) * (Rec1SZ));

        pstLogQueue2 = &stLogQueue2;
        memset(&stLogQueue2, 0x0, sizeof(stLogQueue2));
        stLogQueue2.R_SIZE = Rec2SZ;
        stLogQueue2.Q_SIZE = Que2SZ;

        pstLogQueue2->pstQarr = (structQueueArray*)malloc(sizeof(structQueueArray) * Que2SZ);
        for(j = 0; j < Que2SZ; j++)
            pstLogQueue2->pstQarr[j].Q_BUFFER = (int16*)malloc(sizeof(int16) * (Rec2SZ));

        pstLogQueue3 = &stLogQueue3;
        memset(&stLogQueue3, 0x0, sizeof(stLogQueue3));
        stLogQueue3.R_SIZE = Rec3SZ;
        stLogQueue3.Q_SIZE = Que3SZ;

        pstLogQueue3->pstQarr = (structQueueArray*)malloc(sizeof(structQueueArray) * Que3SZ);
        for(j = 0; j < Que3SZ; j++)
            pstLogQueue3->pstQarr[j].Q_BUFFER = (int16*)malloc(sizeof(int16) * (Rec3SZ));

}


void Log_EnQueue(int16* pPacket){

    int16* pQueue = stLogQueue.pstQarr[ stLogQueue.q_head ].Q_BUFFER;

    memcpy(pQueue, pPacket, pstLogQueue->R_SIZE);

    increase_Queue_value(&pstLogQueue->q_head);
}


void QueueWrite(){

    while(stLogQueue.q_head != stLogQueue.q_tail){

        int16* pQueue = pstLogQueue->pstQarr[ stLogQueue.q_tail ].Q_BUFFER;

        SD_DataStorage(pQueue, pstLogQueue->R_SIZE);

        increase_Queue_value(&pstLogQueue->q_tail);
    }
}

inline void increase_Queue_value(Uint16* data_p){
    (* data_p)++;

    if((* data_p) >= pstLogQueue->Q_SIZE)   (* data_p) = 0x0;
}

void Log_EnQueue2(int16* pPacket){

    int16* pQueue2 = stLogQueue2.pstQarr[ stLogQueue2.q_head ].Q_BUFFER;

    memcpy(pQueue2, pPacket, pstLogQueue2->R_SIZE);

    increase_Queue2_value(&pstLogQueue2->q_head);
}


void Queue2Write(){

    while(stLogQueue2.q_head != stLogQueue2.q_tail){

        int16* pQueue2 = pstLogQueue2->pstQarr[ stLogQueue2.q_tail ].Q_BUFFER;

        SD_DataStorage(pQueue2, pstLogQueue2->R_SIZE);

        increase_Queue2_value(&pstLogQueue2->q_tail);

    }
}

inline void increase_Queue2_value(Uint16* data_p){
    (* data_p)++;

    if((* data_p) >= pstLogQueue2->Q_SIZE)   (* data_p) = 0x0;
}


// �Ʒ� QUEUE3�� ������������ UBLOX SAT�� �ޱ� ���� QUEUE�̴�.
// ������ SDī�忡 �����ϱ� ���� �����ϴ� QUEUE1�� QUEUE2 ���������� �����͸� �����ϴµ� ȿ������ 2���� QUEUE������
// ��������� ������ �����͸� �����ϱ⿡�� ��ȿ�����̴�(������� �ʴ� ���� ���� �� ����).
// ������ ��ü���� �ý��� �� ũ�� ���������� �ʱ� ������ �״�� ����Ѵ�.
// ���߿� ����ȭ�� ���� ������ ������ �˸���.

void Log_EnQueue3(int16* pPacket){

    int16* pQueue3 = stLogQueue3.pstQarr[ stLogQueue3.q_head ].Q_BUFFER;

    Uint16 Size = *(Uint16*)(pPacket + 2);

    memcpy(pQueue3, pPacket, Size);

    increase_Queue3_value(&pstLogQueue3->q_head);
}


void Queue3Write(){

    while(stLogQueue3.q_head != stLogQueue3.q_tail){

        int16* pQueue3 = pstLogQueue3->pstQarr[ stLogQueue3.q_tail ].Q_BUFFER;

        Uint16 Size = *(Uint16*)(pQueue3 + 2);

        SD_DataStorage(pQueue3, Size);

        increase_Queue3_value(&pstLogQueue3->q_tail);

    }
}

inline void increase_Queue3_value(Uint16* data_p){
    (* data_p)++;

    if((* data_p) >= pstLogQueue3->Q_SIZE)   (* data_p) = 0x0;
}
