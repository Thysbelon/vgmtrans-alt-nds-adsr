/**
 * VGMTrans (c) - 2002-2024
 * Licensed under the zlib license
 * See the included LICENSE for more information
 */

#include <cmath>
#include <numeric>
#include <string>
#include <vector>
#include <spdlog/fmt/fmt.h>
#include "Loop.h"
#include "NDSFormat.h"
#include "VGMSampColl.h"
#include "NDSInstrSet.h"
#include "VGMRgn.h"
#include "LogManager.h"
#include "ScaleConversion.h"

// INTR_FREQUENCY is the interval in seconds between updates to the vol for articulation. (INTR_FREQUENCY == 0.005290667. There are ~0.0053 seconds in between each update, or up to ~189 updates per second.)
// In the original software, this is done via an interrupt timer.
// The value was copied from the nocash docs.
// We can multiply the count  by this frequency to find the duration of the articulation phases with
// exact accuracy.
constexpr double INTR_FREQUENCY = 64 * 2728.0 / 33e6;
const float SF2_ADSR_SECONDS_MAX = 101.593; // TODO: figure out if there's a max value that applies to both SF2 and DLS.

// ***********
// NDSInstrSet
// ***********

NDSInstrSet::NDSInstrSet(RawFile *file, uint32_t offset, uint32_t length, VGMSampColl *psg_samples,
                         std::string name)
    : VGMInstrSet(NDSFormat::name, file, offset, length, std::move(name)),
      m_psg_samples(psg_samples) {}

bool NDSInstrSet::parseInstrPointers() {
  uint32_t nInstruments = readWord(dwOffset + 0x38);
  VGMHeader *instrptrHdr = addHeader(dwOffset + 0x38, nInstruments * 4 + 4, "Instrument Pointers");

  for (uint32_t i = 0; i < nInstruments; i++) {
    uint32_t instrPtrOff = dwOffset + 0x3C + i * 4;
    uint32_t temp = readWord(instrPtrOff);
    if (temp == 0) {
      continue;
    }

    uint8_t instrType = temp & 0xFF;
    uint32_t pInstr = temp >> 8;
    aInstrs.push_back(new NDSInstr(this, pInstr + dwOffset, 0, 0, i, instrType));

    VGMHeader *hdr = instrptrHdr->addHeader(instrPtrOff, 4, "Pointer");
    hdr->addChild(instrPtrOff, 1, "Type");
    hdr->addChild(instrPtrOff + 1, 3, "Offset");
  }

  return true;
}

// ********
// NDSInstr
// ********

NDSInstr::NDSInstr(NDSInstrSet *instrSet, uint32_t offset, uint32_t length, uint32_t theBank,
                   uint32_t theInstrNum, uint8_t theInstrType)
    : VGMInstr(instrSet, offset, length, theBank, theInstrNum, "Instrument", 0), instrType(theInstrType) {
}

bool NDSInstr::loadInstr() {
  // All of the undefined case values below are used for tone or noise channels
  switch (instrType) {
    case 0x01: {
      setName("Single-Region Instrument");
      unLength = 10;

      VGMRgn *rgn = addRgn(dwOffset, 10, readShort(dwOffset));
      getSampCollPtr(rgn, readShort(dwOffset + 2));
      getArticData(rgn, dwOffset + 4);
      rgn->addChild(dwOffset + 2, 2, "Sample Collection Index");
      break;
    }

    case 0x02: {
      /* PSG Tone */
      uint8_t dutyCycle = readByte(dwOffset) & 0x07;
      std::string dutyCycles[8] = {"12.5%", "25%", "37.5%", "50%",
                                    "62.5%", "75%", "87.5%", "0%"};
      setName("PSG Wave (" + dutyCycles[dutyCycle] + ")");
      unLength = 10;

      VGMRgn *rgn = addRgn(dwOffset, 10, dutyCycle);
      getArticData(rgn, dwOffset + 4);

      rgn->sampCollPtr = static_cast<NDSInstrSet*>(parInstrSet)->m_psg_samples;
      /* We have to set this manually as all of our samples are generated at 440Hz (69 = A4) */
      rgn->setUnityKey(69);
      break;
    }

    case 0x03: {
      setName("PSG Noise");
      unLength = 10;

      /* The noise sample is the 8th in our PSG sample collection */
      VGMRgn *rgn = addRgn(dwOffset, 10, 8);
      getArticData(rgn, dwOffset + 4);

      rgn->sampCollPtr = static_cast<NDSInstrSet*>(parInstrSet)->m_psg_samples;
      rgn->setUnityKey(45);

      break;
    }

    case 0x10: {
      setName("Drumset");

      uint8_t lowKey = readByte(dwOffset);
      uint8_t highKey = readByte(dwOffset + 1);
      uint8_t nRgns = (highKey - lowKey) + 1;
      for (uint8_t i = 0; i < nRgns; i++) {
        u32 rgnOff = dwOffset + 2 + i * 12;
        VGMRgn *rgn = addRgn(rgnOff, 12, readShort(rgnOff + 2),
                             lowKey + i, lowKey + i);
        getSampCollPtr(rgn, readShort(rgnOff + 4));
        getArticData(rgn, rgnOff + 6);
        rgn->addChild(rgnOff + 2, 2, "Sample Num");
        rgn->addChild(rgnOff + 4, 2, "Sample Collection Index");
      }
      unLength = 2 + nRgns * 12;

      break;
    }

    case 0x11: {
      setName("Multi-Region Instrument");
      uint8_t keyRanges[8];
      uint8_t nRgns = 0;
      for (int i = 0; i < 8; i++) {
        keyRanges[i] = readByte(dwOffset + i);
        if (keyRanges[i] != 0) {
          nRgns++;
        } else {
          break;
        }
        addChild(dwOffset + i, 1, "Key Range");
      }

      for (int i = 0; i < nRgns; i++) {
        u32 rgnOff = dwOffset + 8 + i * 12;
        VGMRgn *rgn = addRgn(rgnOff, 12, readShort(rgnOff + 2),
                             (i == 0) ? 0 : keyRanges[i - 1] + 1, keyRanges[i]);
        getSampCollPtr(rgn, readShort(rgnOff + 4));
        getArticData(rgn, rgnOff + 6);
        addChild(rgnOff + 4, 2, "Sample Collection Index");
      }
      unLength = nRgns * 12 + 8;

      break;
    }
    default:
      break;
  }
  return true;
}

void NDSInstr::getSampCollPtr(VGMRgn *rgn, int waNum) const {
  rgn->sampCollPtr = static_cast<NDSInstrSet*>(parInstrSet)->sampCollWAList[waNum];
}

void NDSInstr::getArticData(VGMRgn *rgn, uint32_t offset) const { // TODO: get rid of duplicate code. Replace while loops with for loops. Improve variable names.
  // Code was copied then converted to C++ from https://github.com/DaforLynx/adsr_calculator/blob/master/src/main.rs#L66 .
  const double ZERO_VOL = -92544;
  const uint8_t ATTACK_TABLE[] = {
    255, 254, 253, 252, 251, 250, 249, 248, 247, 246, 245, 244, 243, 242, 241, 240,
    239, 238, 237, 236, 235, 234, 233, 232, 231, 230, 229, 228, 227, 226, 225, 224,
    223, 222, 221, 220, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 209, 208,
    207, 206, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193, 192,
    191, 190, 189, 188, 187, 186, 185, 184, 183, 182, 181, 180, 179, 178, 177, 176,
    175, 174, 173, 172, 171, 170, 169, 168, 167, 166, 165, 164, 163, 162, 161, 160,
    159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 149, 148, 147, 143, 137, 132,
    127, 123, 116, 109, 100, 92, 84, 73, 63, 51, 38, 26, 14, 5, 1, 0,
  };
  const uint16_t DECAY_TABLE[] = {
    1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43,
    45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85,
    87, 89, 91, 93, 95, 97, 99, 101, 102, 104, 105, 107, 108, 110, 111, 113, 115, 116,
    118, 120, 122, 124, 126, 128, 130, 132, 135, 137, 140, 142, 145, 148, 151, 154,
    157, 160, 163, 167, 171, 175, 179, 183, 187, 192, 197, 202, 208, 213, 219, 226,
    233, 240, 248, 256, 265, 274, 284, 295, 307, 320, 334, 349, 366, 384, 404, 427,
    452, 480, 512, 549, 591, 640, 698, 768, 853, 960, 1097, 1280, 1536, 1920, 2560,
    3840, 7680, 15360, 65535,
  };
  const double SUSTAIN_TABLE[] = {
    -92544, -92416, -92288, -83328, -76928, -71936, -67840, -64384, -61440, -58880,
    -56576, -54400, -52480, -50688, -49024, -47488, -46080, -44672, -43392, -42240,
    -41088, -40064, -39040, -38016, -36992, -36096, -35328, -34432, -33664, -32896,
    -32128, -31360, -30592, -29952, -29312, -28672, -28032, -27392, -26880, -26240,
    -25728, -25088, -24576, -24064, -23552, -23040, -22528, -22144, -21632, -21120,
    -20736, -20224, -19840, -19456, -19072, -18560, -18176, -17792, -17408, -17024,
    -16640, -16256, -16000, -15616, -15232, -14848, -14592, -14208, -13952, -13568,
    -13184, -12928, -12672, -12288, -12032, -11648, -11392, -11136, -10880, -10496,
    -10240, -9984, -9728, -9472, -9216, -8960, -8704, -8448, -8192, -7936, -7680,
    -7424, -7168, -6912, -6656, -6400, -6272, -6016, -5760, -5504, -5376, -5120, -4864,
    -4608, -4480, -4224, -3968, -3840, -3584, -3456, -3200, -2944, -2816, -2560, -2432,
    -2176, -2048, -1792, -1664, -1408, -1280, -1024, -896, -768, -512, -384, -128, 0,
  };

  rgn->addUnityKey(readByte(offset), offset, 1);
  uint8_t AttackTime = readByte(offset + 1);
  uint8_t DecayTime = readByte(offset + 2);
  uint8_t SustainLev = readByte(offset + 3);
  uint8_t ReleaseTime = readByte(offset + 4);
  uint8_t Pan = readByte(offset + 5);

  rgn->addADSRValue(offset + 1, 1, "Attack Rate");
  rgn->addADSRValue(offset + 2, 1, "Decay Rate");
  rgn->addADSRValue(offset + 3, 1, "Sustain Level");
  rgn->addADSRValue(offset + 4, 1, "Release Time");
  rgn->addChild(offset + 5, 1, "Pan");

  double realAttack = 0;
  double vel = ZERO_VOL;
  uint32_t steps = 0; // cycles

  if (AttackTime != 0) {
    while (vel < -0.00001) { // fix -0.000000 weirdness. TODO: after changing all floats to doubles, check if the negative zero weirdness still happens; if not, change -0.00001 back to 0.
      steps += 1;
      vel = ATTACK_TABLE[AttackTime] * vel / 0xff;
      if (steps >= 0xFFFFFF) {
        printf("NDSInstrSet.cpp: attack while loop had to be manually broken. vel: %f\n", vel);
      }
    }
    realAttack = static_cast<double>(steps) / (1.0 / INTR_FREQUENCY);
  } else {
    realAttack = SF2_ADSR_SECONDS_MAX;
  }
  rgn->attack_time = realAttack;

  vel = 0; // should be max volume for calculating decay. "0" is max volume for nds.
  steps = 0;

  double realDecay = 0;
  while (vel > ZERO_VOL) {
    steps += 1;
    vel -= DECAY_TABLE[DecayTime];
    if (steps >= 0xFFFFFF) {
      printf("NDSInstrSet.cpp: decay while loop had to be manually broken. vel: %f\n", vel);
    }
  }
  realDecay = static_cast<double>(steps) / (1.0 / INTR_FREQUENCY);

  rgn->decay_time = realDecay;

  double realSustain = 0;
  if (SustainLev == 0) {
    //127.0
    //realSustain = 0;
    realSustain = dbToAmp(127.0); // dbToAmp is from ScaleConversion in util.
  } else {
    double sus = SUSTAIN_TABLE[(127 - SustainLev)];
    double amplitude = sus / ZERO_VOL; // 0 is 1.0, 127 is 0.0
    double decibels = (20.0 * std::log10(std::abs(amplitude))) / 2.0; // For some reason having a less prominent sustain difference tends to sound more accurate
    realSustain = dbToAmp(std::abs(decibels));
    //decibels.abs() // Written as "decibels to diminish by" in Polyphone
  }
  rgn->sustain_level = realSustain;

  vel = 0;
  steps = 0;

  double realRelease = 0;
  while (vel > ZERO_VOL) {
    steps += 1;
    vel -= DECAY_TABLE[ReleaseTime];
    if (steps >= 0xFFFFFF) {
      printf("NDSInstrSet.cpp: release while loop had to be manually broken. vel: %f\n", vel);
    }
  }
  realRelease = static_cast<double>(steps) / (1.0 / INTR_FREQUENCY);

  rgn->release_time = realRelease;

  if (Pan == 0)
    rgn->pan = 0;
  else if (Pan == 127)
    rgn->pan = 1.0;
  else if (Pan == 64)
    rgn->pan = 0.5;
  else
    rgn->pan = static_cast<double>(Pan) / 127;
}

// ***********
// NDSWaveArch
// ***********

NDSWaveArch::NDSWaveArch(RawFile *file, uint32_t offset, uint32_t length, std::string name)
    : VGMSampColl(NDSFormat::name, file, offset, length, std::move(name)) {
}

bool NDSWaveArch::parseHeader() {
  unLength = readWord(dwOffset + 8);
  return true;
}

bool NDSWaveArch::parseSampleInfo() {
  uint32_t nSamples = readWord(dwOffset + 0x38);
  for (uint32_t i = 0; i < nSamples; i++) {
    uint32_t pSample = readWord(dwOffset + 0x3C + i * 4) + dwOffset;
    int nChannels = 1;
    uint8_t waveType = readByte(pSample);
    bool bLoops = (readByte(pSample + 1) != 0);
    uint16_t rate = readShort(pSample + 2);
    uint16_t bps;
    // uint8_t multiplier;
    switch (waveType) {
      case NDSSamp::PCM8:
        bps = 8;
        break;
      case NDSSamp::PCM16:
        bps = 16;
        break;
      case NDSSamp::IMA_ADPCM:
        bps = 16;
        break;
      default:
        L_ERROR("Parsed invalid wave type: {}", waveType);
        bps = 16;
        break;
    }
    uint32_t loopOff =
        (readShort(pSample + 6)) *
        4;  //*multiplier; //represents loop point in words, excluding header supposedly
    uint32_t nonLoopLength =
        readShort(pSample + 8) * 4;  // if IMA-ADPCM, subtract one for the ADPCM header

    uint32_t dataStart, dataLength;
    if (waveType == NDSSamp::IMA_ADPCM) {
      dataStart = pSample + 0x10;
      dataLength = loopOff + nonLoopLength - 4;
    } else {
      dataStart = pSample + 0xC;
      dataLength = loopOff + nonLoopLength;
    }

    auto name = fmt::format("Sample {}", samples.size());
    NDSSamp *samp = new NDSSamp(this, pSample, dataStart + dataLength - pSample, dataStart,
                                dataLength, nChannels, bps, rate, waveType, name);

    if (waveType == NDSSamp::IMA_ADPCM) {
      samp->setLoopStartMeasure(LM_SAMPLES);
      samp->setLoopLengthMeasure(LM_SAMPLES);
      loopOff *= 2;               // now it's in samples
      loopOff = loopOff - 8 + 1;  // exclude the header's sample.  not exactly sure why 8.
      nonLoopLength = (dataLength * 2 + 1) - loopOff;
      samp->ulUncompressedSize = (nonLoopLength + loopOff) * 2;
    }

    samp->setLoopStatus(bLoops);
    samp->setLoopOffset(loopOff);
    samp->setLoopLength(nonLoopLength);
    samples.push_back(samp);
  }
  return true;
}

NDSPSG::NDSPSG(RawFile *file) : VGMSampColl(NDSFormat::name, file, 0, 0, "NDS PSG samples") {
}

bool NDSPSG::parseSampleInfo() {
  /* 8 waves + noise */
  for (uint8_t i = 0; i <= 8; i++) {
    samples.push_back(new NDSPSGSamp(this, i));
  }

  return true;
}

// *******
// NDSSamp
// *******

NDSSamp::NDSSamp(VGMSampColl *sampColl, uint32_t offset, uint32_t length, uint32_t dataOffset,
                 uint32_t dataLen, uint8_t nChannels, uint16_t theBPS, uint32_t theRate,
                 uint8_t theWaveType, std::string name)
    : VGMSamp(sampColl, offset, length, dataOffset, dataLen, nChannels, theBPS, theRate, std::move(name)),
      waveType(theWaveType) {
}

double NDSSamp::compressionRatio() {
  if (waveType == IMA_ADPCM) {
    return 4.0;
  }

  return 1.0;
}

void NDSSamp::convertToStdWave(uint8_t *buf) {
  if (waveType == IMA_ADPCM) {
    convertImaAdpcm(buf);
  } else if (waveType == PCM8) {
    readBytes(dataOff, dataLength, buf);
  } else {
    readBytes(dataOff, dataLength, buf);
  }
}

// From nocash's site: The NDS data consist of a 32bit header, followed by 4bit values (so each byte
// contains two values, the first value in the lower 4bits, the second in upper 4 bits). The 32bit
// header contains initial values:
//
//  Bit0-15   Initial PCM16 Value (Pcm16bit = -7FFFh..+7FFF) (not -8000h)
//  Bit16-22  Initial Table Index Value (Index = 0..88)
//  Bit23-31  Not used (zero)

// As far as I can tell, the NDS IMA-ADPCM format has one difference from standard IMA-ADPCM:
// it clamps min (and max?) sample values differently (see below).  I really don't know how much of
// a difference it makes, but this implementation is, to my knowledge, the proper way of doing
// things for NDS.
void NDSSamp::convertImaAdpcm(uint8_t *buf) {
  uint32_t destOff = 0;
  uint32_t sampHeader = getWord(dataOff - 4);
  int decompSample = sampHeader & 0xFFFF;
  int stepIndex = (sampHeader >> 16) & 0x7F;
  // int decompSample = GetShort(dataOff);
  // int stepIndex = GetShort(dataOff+2);
  uint32_t curOffset = dataOff;
  ((int16_t *)buf)[destOff++] = (int16_t)decompSample;

  uint8_t compByte;
  while (curOffset < dataOff + dataLength) {
    compByte = readByte(curOffset++);
    process_nibble(compByte, stepIndex, decompSample);
    ((int16_t *)buf)[destOff++] = (int16_t)decompSample;
    process_nibble((compByte & 0xF0) >> 4, stepIndex, decompSample);
    ((int16_t *)buf)[destOff++] = (int16_t)decompSample;
  }
}

// I'm copying nocash's IMA-ADPCM conversion method verbatim.  Big thanks to him.
// Info is at http://nocash.emubase.de/gbatek.htm#dssound and the algorithm is described as follows:
//
// The NDS data consist of a 32bit header, followed by 4bit values (so each byte contains two
// values, the first value in the lower 4bits, the second in upper 4 bits). The 32bit header
// contains initial values:
//
//  Bit0-15   Initial PCM16 Value (Pcm16bit = -7FFFh..+7FFF) (not -8000h)
//  Bit16-22  Initial Table Index Value (Index = 0..88)
//  Bit23-31  Not used (zero)
//
// In theory, the 4bit values are decoded into PCM16 values, as such:
//
//  Diff = ((Data4bit AND 7)*2+1)*AdpcmTable[Index]/8      ;see rounding-error
//  IF (Data4bit AND 8)=0 THEN Pcm16bit = Max(Pcm16bit+Diff,+7FFFh)
//  IF (Data4bit AND 8)=8 THEN Pcm16bit = Min(Pcm16bit-Diff,-7FFFh)
//  Index = MinMax (Index+IndexTable[Data4bit AND 7],0,88)
//
// In practice, the first line works like so (with rounding-error):
//
//  Diff = AdpcmTable[Index]/8
//  IF (data4bit AND 1) THEN Diff = Diff + AdpcmTable[Index]/4
//  IF (data4bit AND 2) THEN Diff = Diff + AdpcmTable[Index]/2
//  IF (data4bit AND 4) THEN Diff = Diff + AdpcmTable[Index]/1
//
// And, a note on the second/third lines (with clipping-error):
//
//  Max(+7FFFh) leaves -8000h unclipped (can happen if initial PCM16 was -8000h)
//  Min(-7FFFh) clips -8000h to -7FFFh (possibly unlike windows .WAV files?)

#define IMAMax(samp) (samp > 0x7FFF) ? ((short)0x7FFF) : samp
#define IMAMin(samp) (samp < -0x7FFF) ? ((short)-0x7FFF) : samp
#define IMAIndexMinMax(index, min, max) (index > max) ? max : ((index < min) ? min : index)

void NDSSamp::process_nibble(unsigned char data4bit, int &Index, int &Pcm16bit) {
  // int Diff = ((Data4bit & 7)*2+1)*AdpcmTable[Index]/8;
  int Diff = AdpcmTable[Index] / 8;
  if (data4bit & 1)
    Diff = Diff + AdpcmTable[Index] / 4;
  if (data4bit & 2)
    Diff = Diff + AdpcmTable[Index] / 2;
  if (data4bit & 4)
    Diff = Diff + AdpcmTable[Index] / 1;

  if ((data4bit & 8) == 0)
    Pcm16bit = IMAMax(Pcm16bit + Diff);
  if ((data4bit & 8) == 8)
    Pcm16bit = IMAMin(Pcm16bit - Diff);
  Index = IMAIndexMinMax(Index + IMA_IndexTable[data4bit & 7], 0, 88);
}

void NDSSamp::clamp_step_index(int &stepIndex) {
  if (stepIndex < 0)
    stepIndex = 0;
  if (stepIndex > 88)
    stepIndex = 88;
}

void NDSSamp::clamp_sample(int &decompSample) {
  if (decompSample < -32768)
    decompSample = -32768;
  if (decompSample > 32767)
    decompSample = 32767;
}

NDSPSGSamp::NDSPSGSamp(VGMSampColl *sampcoll, uint8_t duty_cycle) : VGMSamp(sampcoll) {
  switch (duty_cycle) {
    case 7: {
      m_duty_cycle = 0;
      break;
    }
    case 0: {
      m_duty_cycle = 0.125;
      break;
    }
    case 1: {
      m_duty_cycle = 0.25;
      break;
    }
    case 2: {
      m_duty_cycle = 0.375;
      break;
    }
    case 3: {
      m_duty_cycle = 0.5;
      break;
    }
    case 4: {
      m_duty_cycle = 0.625;
      break;
    }
    case 5: {
      m_duty_cycle = 0.75;
      break;
    }
    case 6: {
      m_duty_cycle = 0.875;
      break;
    }
    default: {
      /* We don't care about the rest */
      break;
    }
  }

  setNumChannels(1);
  /* This is the NDS mixer frequency */
  setRate(32768);
  setBPS(16);
  setWaveType(WT_PCM16);

  setLoopStatus(true);

  setLoopOffset(0);
  setLoopLength(32768);
  setLoopStartMeasure(LM_SAMPLES);
  setLoopLengthMeasure(LM_SAMPLES);
  ulUncompressedSize = 32768 * bps / 8;

  setName("PSG_duty_" + std::to_string(duty_cycle));
}

void NDSPSGSamp::convertToStdWave(uint8_t *buf) {
  /* Give that the wave type is PCM-16, this is handy */
  int16_t *output = reinterpret_cast<int16_t *>(buf);

  /* Noise mode */
  if (m_duty_cycle == -1) {
    int16_t value = 0x7FFF;
    output[0] = 0x7FFF;
    for (int i = 1, len = loopLength(); i < len; i++) {
      bool carry = value & 0x0001;
      value >>= 1;
      if (carry) {
        output[i] = -0x7FFF;
        value ^= 0x6000;
      } else {
        output[i] = 0x7FFF;
      }
    }
  } else {
    /*
     * PSG wave mode
     * It's band limited so it should sound nice!
     */

    /* Generate Fourier coefficients */
    std::vector<double> coefficients = {m_duty_cycle - 0.5};
    {
      int i = 1;
      std::generate_n(std::back_inserter(coefficients), rate / (440 * 2),
                      [duty_cycle = m_duty_cycle, &i]() {
                        double val = sin(i * duty_cycle * M_PI) * 2 / (i * M_PI);
                        i++;

                        return val;
                      });
    }

    /* Generate audio */
    double scale = 440 * M_PI * 2 / rate;
    for (int i = 0, len = loopLength(); i < len; i++) {
      int counter = 0;
      double value = std::accumulate(std::begin(coefficients), std::end(coefficients), 0.0,
                                     [i, scale, &counter](double sum, double coef) {
                                       sum += coef * cos(counter++ * scale * i);
                                       return sum;
                                     });

      /* We have to go from F64 to S16 */
      int16_t out_value = static_cast<int16_t>(std::round(value * 0x7FFF));
      output[i] = out_value;
    }
  }
}
