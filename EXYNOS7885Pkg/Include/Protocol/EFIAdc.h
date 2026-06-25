#ifndef _EFI_ADC_H_
#define _EFI_ADC_H_

/**
  This Function Reads the current raw ADC value from the selected channel.

  @param[in] Channel                       - The ADC Channel.

  @return The current raw ADC value from the selected channel, or -1 if the read fails.
**/
typedef
INTN
(EFIAPI *EFI_ADC_READ_RAW) (
  IN UINT32 Channel
  );

//
// Define Protocol
//
typedef struct {
  EFI_ADC_READ_RAW ReadRaw;
} EFI_ADC_PROTOCOL;

#endif /* _EFI_ADC_H_ */