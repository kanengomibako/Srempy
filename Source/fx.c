#include "main.h"

extern uint8_t sw[];
extern uint8_t mute;

/* �G�t�F�N�g�I���@���������邱�Ƃ��ł���  -----------------------------------------------*/
static float fx(float x)
{
  x = p3(x);
  return x;
}

/* �o�C�p�X �|�b�v�m�C�Y�΍�̂��߁A0.01�����ʑ��삵�G�t�F�N�g�؂�ւ�����----------------*/
float bypass(float x)
{
  static uint16_t count = 0;
  
  if (mute == 1) return 0.0f; // �f�[�^�ۑ����m�C�Y���o�邽�߃~���[�g����
  
  if ( sw[0] != 0 ) // �G�t�F�N�gON
  {
    if (count < 100) // �o�C�p�X���ʃ_�E��
    {
      x = (1.0f - (float) count * 0.01f) * x;
      count++;
    }
    else if (count < 200) // �G�t�F�N�g���ʃA�b�v
    {
      x = ((float) count * 0.01f - 1.0f) * fx(x);
      count++;
    }
    else // count�I�� (count: 200)
    {
      x = fx(x);
    }
  }
  else // �G�t�F�N�gOFF
  {
    if (count > 100) // �G�t�F�N�g���ʃ_�E��
    {
      x = ((float) count * 0.01f - 1.0f) * fx(x);
      count--;
    }
    else if ( count > 0) // �o�C�p�X���ʃA�b�v
    {
      x = (1.0f - (float) count * 0.01f) * x;
      count--;
    }
  }
  return x;
}

/* BiQuad�t�B���^�W�� �ߎ��v�Z ----------------------------------------------------------*/
float bqSinOmega(float fc)
{
  float fc2 = fc * fc;
  return fc * ( 0.0001308996939f - 3.738218157e-13f * fc2 + 3.202667914e-22f * fc2 * fc2 );
  //return sinf( 2.0f * PI * fc / fs );
}

float bqCosOmega(float fc)
{
  float fc2 = fc * fc;
  return 1.0f - 8.567364932e-9f * fc2 + 1.223329031e-17f * fc2 * fc2;
  //return cosf( 2.0f * PI * fc / fs );
}

float bqAlphaQ(float fc,float q)
{
  return bqSinOmega(fc) / ( 2.0f * q );
}

float bqAlphaBW(float fc,float bw)
{
  return ( 4.66355e-6f * bw * bw + 4.15256e-5f * bw + 7.04497e-7f) * fc;
}

float bqA(float gain)
{
  return expf( 0.05756462732f * gain );
}

float bqBeta(float gain,float q)
{
  return expf( 0.02878231366f * gain ) / q;
}
