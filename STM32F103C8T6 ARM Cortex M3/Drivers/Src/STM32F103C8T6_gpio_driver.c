/*
 * STM32F103C8T6_gpio_driver.c
 *
 *  Created on: Feb 10, 2026
 *      Author: Owner
 */

#include "STM32F103C8T6_gpio_driver.h"
#include "STM32F103C8T6.h"
/*
 * PERIPHERAL CLOCK SET UP
 */
/**
 ******************************************************************************
 * @fn      void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
 * @brief   Enables or disables peripheral clock for the given GPIO port.
 *
 * @param   [in] pGPIOx   Base address of the GPIO peripheral (GPIOA, GPIOB, ...)
 * @param   [in] EnOrDi   ENABLE or DISABLE macro
 *
 * @return  None
 *
 * @note    Must be called before using any GPIO APIs.
 ******************************************************************************
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }

    }
    else  // DISABLE
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }

    }
}
/*
 * INIT AND DEINIT
 */
/**
 ******************************************************************************
 * @fn      void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
 * @brief   Khởi tạo và cấu hình một chân GPIO theo tham số truyền vào.
 *
 * @param   [in] pGPIOHandle  Con trỏ đến cấu trúc GPIO_Handle_t chứa:
 *                            - PinNumber: số thứ tự chân (0..15)
 *                            - Mode: chế độ hoạt động (input, output, interrupt…)
 *                            - CNF: cấu hình chi tiết (push-pull, open-drain, pull-up/down…)
 *                            - pGPIOx: base address của peripheral GPIO (GPIOA, GPIOB…)
 *
 * @return  None
 *
 * @note    Hàm này thực hiện:
 *          1. Cấu hình thanh ghi CRL/CRH để thiết lập mode và CNF cho chân GPIO.
 *          2. Nếu chọn chế độ ngắt (IT_FT, IT_RT, IT_RFT):
 *             - Enable clock cho AFIO.
 *             - Map chân GPIO sang EXTI line tương ứng qua AFIO->EXTICR.
 *             - Thiết lập trigger (falling, rising, hoặc cả hai).
 *             - Unmask line trong EXTI->IMR để cho phép ngắt.
 *          3. Bước cấu hình NVIC (kết nối ngắt với CPU) sẽ được thực hiện
 *             ở API riêng (GPIO_IRQ_Interupt_Config).
 ******************************************************************************
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    uint32_t pos = pGPIOHandle->PinNumber;
    uint32_t config = (pGPIOHandle->Mode & 0x3) | ((pGPIOHandle->CNF & 0x3) << 2);

    // Cấu hình CRL/CRH cho input/output
    if(pos < 8) {
        pGPIOHandle->pGPIOx->CRL &= ~(0xF << (pos * 4));
        pGPIOHandle->pGPIOx->CRL |=  (config << (pos * 4));
    } else {
        pos -= 8;
        pGPIOHandle->pGPIOx->CRH &= ~(0xF << (pos * 4));
        pGPIOHandle->pGPIOx->CRH |=  (config << (pos * 4));
    }

    // Nếu chọn mode interrupt thì cấu hình EXTI
    if(pGPIOHandle->Mode == GPIO_MODE_IT_FT ||
       pGPIOHandle->Mode == GPIO_MODE_IT_RT ||
       pGPIOHandle->Mode == GPIO_MODE_IT_RFT)
    {
    	// ở đây đọc reference và manual thì config exti nằm ở AFIO thuộc bus ABP2 đã define địa chỉ trong .h
    	// Lần lượt theo các bước
        // 1. Enable AFIO clock
        AFIO_PCLK_EN();
        // 2. Map pin sang EXTI line => chọn port GPIx1,2,3,....
        uint8_t exti_index = pGPIOHandle->PinNumber / 4;
        uint8_t exti_pos   = (pGPIOHandle->PinNumber % 4) * 4;
        AFIO->EXTICR[exti_index] &= ~(0xF << exti_pos); // reset
        AFIO->EXTICR[exti_index] |= (GPIO_PORTCODE(pGPIOHandle->pGPIOx) << exti_pos); // write giá trị

        // 3. Chọn trigger hay là mode
        if(pGPIOHandle->Mode == GPIO_MODE_IT_FT) {
            EXTI->FTSR |= (1 << pGPIOHandle->PinNumber);
            EXTI->RTSR &= ~(1 << pGPIOHandle->PinNumber);
        }
        else if(pGPIOHandle->Mode == GPIO_MODE_IT_RT){
            EXTI->RTSR |= (1 << pGPIOHandle->PinNumber);
            EXTI->FTSR &= ~(1 << pGPIOHandle->PinNumber);
        }
        else if(pGPIOHandle->Mode == GPIO_MODE_IT_RFT) {
            EXTI->FTSR |= (1 << pGPIOHandle->PinNumber);
            EXTI->RTSR |= (1 << pGPIOHandle->PinNumber);
        }

        // 4. Unmask line, gỡ block các đường bi chắn bởi exti
        EXTI->IMR |= (1 << pGPIOHandle->PinNumber);
   // 4 bước trên ta config peripheral còn bước cuối cùng mình phải config phần NVIC để kết nối với MCU xử lí
        // bước NVIC ta sẽ làm ở API IQR_CONFIG
}
}

/* the example to use
GPIO_Handle_t gpioA5;
gpioA5.pGPIOx     = GPIOA;
gpioA5.PinNumber  = GPIO_PIN_5;
gpioA5.Mode       = GPIO_MODE_OUTPUT_50MHZ;
gpioA5.CNF        = GPIO_CNF_OUTPUT_PP;
GPIO_Init(&gpioA5);
*/
/**
 ******************************************************************************
 * @fn      void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
 * @brief   Reset lại toàn bộ cấu hình của một peripheral GPIO về trạng thái mặc định.
 *
 * @param   [in] pGPIOx   Base address của peripheral GPIO (GPIOA, GPIOB, GPIOC, ...)
 *
 * @return  None
 *
 * @note    - Hàm này sẽ gọi macro reset tương ứng (GPIOx_REG_RESET) để đưa
 *            peripheral về trạng thái ban đầu như sau khi reset hệ thống.
 *          - Sau khi gọi hàm này, tất cả các cấu hình pin (Mode, CNF, ODR, ...)
 *            sẽ trở về giá trị mặc định.
 *          - Cần khởi tạo lại GPIO bằng hàm GPIO_Init trước khi sử dụng.
 ******************************************************************************
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	        {
		GPIOA_REG_RESET();
	        }
	        else if(pGPIOx == GPIOB)
	        {
	        	GPIOB_REG_RESET();
	        }
	        else if(pGPIOx == GPIOC)
	        {
	        	GPIOC_REG_RESET();
	        }
	        else if(pGPIOx == GPIOD)
	        {
	        	GPIOD_REG_RESET();
	        }
	        else if(pGPIOx == GPIOE)
	        {
	        	GPIOE_REG_RESET();
	        }
}
/*
 * DATA READ AND WRITE
 */
/**
 ******************************************************************************
 * @fn      uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
 * @brief   Đọc trạng thái logic của một chân GPIO cụ thể.
 *
 * @param   [in] pGPIOx     Base address của peripheral GPIO (GPIOA, GPIOB, ...)
 * @param   [in] PinNumber  Số thứ tự chân cần đọc (0..15)
 *
 * @return  Trả về giá trị 0 hoặc 1 tương ứng với mức logic LOW hoặc HIGH
 *
 * @note    - Hàm này đọc bit tương ứng trong thanh ghi IDR (Input Data Register).
 *          - Thích hợp để kiểm tra trạng thái của một chân input riêng lẻ.
 ******************************************************************************
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
/**
 ******************************************************************************
 * @fn      uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
 * @brief   Đọc toàn bộ trạng thái logic của port GPIO.
 *
 * @param   [in] pGPIOx   Base address của peripheral GPIO (GPIOA, GPIOB, ...)
 *
 * @return  Trả về giá trị 16-bit, mỗi bit biểu diễn trạng thái của một chân
 *          (bit0 → Pin0, bit1 → Pin1, ... bit15 → Pin15).
 *
 * @note    - Hàm này đọc toàn bộ thanh ghi IDR (Input Data Register).
 *          - Thích hợp để lấy snapshot trạng thái của cả port cùng lúc.
 ******************************************************************************
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
		value = (uint16_t)(pGPIOx->IDR);
		return value;
}
/**
 ******************************************************************************
 * @fn      void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
 * @brief   Ghi giá trị logic (SET/RESET) cho một chân GPIO cụ thể.
 *
 * @param   [in] pGPIOx     Base address của peripheral GPIO (GPIOA, GPIOB, ...)
 * @param   [in] PinNumber  Số thứ tự chân cần ghi (0..15)
 * @param   [in] Value      Giá trị cần ghi:
 *              - GPIO_PIN_SET   : ghi mức logic HIGH (1)
 *              - GPIO_PIN_RESET : ghi mức logic LOW  (0)
 *
 * @return  None
 *
 * @note    - Hàm này thao tác trực tiếp trên thanh ghi ODR (Output Data Register).
 *          - Thích hợp để điều khiển trạng thái của một chân output riêng lẻ.
 ******************************************************************************
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
  if (Value == GPIO_PIN_SET){
	  pGPIOx->ODR |= (1 << PinNumber);
  } else
  {
  pGPIOx ->ODR &= ~(1 << PinNumber);
  }
}
/**
 ******************************************************************************
 * @fn      void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
 * @brief   Ghi toàn bộ giá trị cho port GPIO.
 *
 * @param   [in] pGPIOx   Base address của peripheral GPIO (GPIOA, GPIOB, ...)
 * @param   [in] Value    Giá trị 16-bit cần ghi vào port (bit0 → Pin0, bit1 → Pin1, ...)
 *
 * @return  None
 *
 * @note    - Hàm này ghi trực tiếp toàn bộ thanh ghi ODR (Output Data Register).
 *          - Thích hợp để thiết lập trạng thái đồng thời cho nhiều chân GPIO.
 ******************************************************************************
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
/**
 ******************************************************************************
 * @fn      void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
 * @brief   Đảo trạng thái logic của một chân GPIO cụ thể.
 *
 * @param   [in] pGPIOx     Base address của peripheral GPIO (GPIOA, GPIOB, ...)
 * @param   [in] PinNumber  Số thứ tự chân cần đảo trạng thái (0..15)
 *
 * @return  None
 *
 * @note    - Hàm này thực hiện phép XOR trên thanh ghi ODR (Output Data Register).
 *          - Nếu pin đang ở mức HIGH → chuyển sang LOW.
 *          - Nếu pin đang ở mức LOW  → chuyển sang HIGH.
 *          - Thích hợp để điều khiển LED hoặc các thiết bị cần đảo trạng thái nhanh.
 ******************************************************************************
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);

}
/*
 * IRQ CONFIGURATION AND ISR HANDLING
 */
/**
 ******************************************************************************
 * @fn      void GPIO_IRQ_Interupt_Config(uint8_t IRQNumber, uint8_t EnorDi)
 * @brief   Bật hoặc tắt một ngắt cụ thể trong NVIC.
 *
 * @param   [in] IRQNumber  Số thứ tự của ngắt cần cấu hình (theo định nghĩa IRQn).
 * @param   [in] EnorDi     Trạng thái mong muốn:
 *                          - ENABLE: Kích hoạt ngắt
 *                          - DISABLE: Vô hiệu hóa ngắt
 *
 * @return  None
 *
 * @note    Hàm này xác định ngắt thuộc nhóm nào (ISER0, ISER1, ISER2) dựa trên
 *          giá trị IRQNumber, sau đó ghi bit tương ứng để bật/tắt ngắt.
 *          - IRQNumber 0..31   → ISER0 / ICER0
 *          - IRQNumber 32..63  → ISER1 / ICER1
 *          - IRQNumber 64..95  → ISER2 / ICER2
 ******************************************************************************
 */
void GPIO_IRQ_Interupt_Config(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	    {
	        if(IRQNumber <= 31)
	        {
	            // program ISER0 register
	        	*NVIC_ISER0 |= (1 << IRQNumber);

	        }
	        else if( IRQNumber > 31 && IRQNumber < 64 ) // 32 to 63
	        {
	        	// program ISER1 register
	        	*NVIC_ISER1 |= (1 << IRQNumber % 32);
	        }
	        else if(IRQNumber >= 64 && IRQNumber < 96 ) // 64 to 95
	        {
	        	// program ISER2 register
	        	*NVIC_ISER2 |= (1 << IRQNumber % 64);
	        }
	    }
	    else  // DISABLE
	    {
	        if(IRQNumber <= 31)
	    	        {
	    	            // program ISER0 register
	        	      *NVIC_ICER0  |= (1 << IRQNumber);

	    	        }
	    	        else if( IRQNumber > 31 && IRQNumber < 64 ) // 32 to 63
	    	        {
	    	        	// program ISER1 register
	    	        	*NVIC_ICER1 |= (1 << IRQNumber % 32);
	    	        }
	    	        else if(IRQNumber >= 64 && IRQNumber < 96 )
	    	        {
	    	        	// program ISER2 register
	    	        	*NVIC_ICER2 |= (1 << IRQNumber % 64);
	    	        }

	    }
}
/**
 ******************************************************************************
 * @fn      void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
 * @brief   Cấu hình mức độ ưu tiên cho một ngắt cụ thể trong NVIC.
 *
 * @param   [in] IRQNumber   Số thứ tự của ngắt cần cấu hình (theo định nghĩa IRQn).
 * @param   [in] IRQPriority Mức độ ưu tiên mong muốn (giá trị nhỏ hơn nghĩa là ưu tiên cao hơn).
 *
 * @return  None
 *
 * @note    Hàm này tính toán vị trí trong thanh ghi IPR (Interrupt Priority Register)
 *          tương ứng với IRQNumber, sau đó ghi giá trị ưu tiên vào đúng vị trí bit.
 ******************************************************************************
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	// 1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4 ;
	uint8_t shift_amount = ( iprx_section*8 ) - ( 8- NO_PR_BITS_IMPLEMENT );
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount );
}
/**
 ******************************************************************************
 * @fn      void GPIO_IRQHandling(uint8_t PinNumber)
 * @brief   Xử lý ngắt ngoài (EXTI) cho một chân GPIO cụ thể.
 *
 * @param   [in] PinNumber   Số thứ tự chân GPIO (0..15) được cấu hình làm nguồn ngắt.
 *
 * @return  None
 *
 * @note    Hàm này kiểm tra cờ ngắt trong thanh ghi EXTI->PR. Nếu cờ được set,
 *          nó sẽ được xóa để xác nhận đã xử lý ngắt, tránh lặp lại.
 ******************************************************************************
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
// clear the exti pr register corresponding
	if (EXTI->PR & (1 << PinNumber))
	{
		// clear
		EXTI->PR |= (1 << PinNumber);

	}
}
