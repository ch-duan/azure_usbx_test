/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_usbx_host.c
 * @author  MCD Application Team
 * @brief   USBX host applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_usbx_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "ux_hcd_stm32.h"
#include "ux_host_class_video.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define T_STRING(v) #v
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USBX_MEMORY_SIZE    (64 * 1024)
#define APP_QUEUE_SIZE      1
#define USBX_APP_STACK_SIZE 1024
#define USBX_MEMORY_SIZE    (64 * 1024)
#define VIDEO_BUFFER_NB     (UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST_COUNT - 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD ux_app_thread;
TX_THREAD uvc_app_thread;
TX_QUEUE ux_app_MsgQueue;
UX_HOST_CLASS_VIDEO *video_class;
TX_SEMAPHORE data_received_semaphore;
#if defined(__ICCARM__) /* IAR Compiler */
#pragma data_alignment = 4
#endif /* defined ( __ICCARM__ ) */
__ALIGN_BEGIN ux_app_devInfotypeDef ux_dev_info __ALIGN_END;
uint8_t video_buffer[20 * 1024] __attribute__((section(".sram")));
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void Error_Handler(void);
extern void MX_USB_OTG_HS_HCD_Init(void);
extern HCD_HandleTypeDef hhcd_USB_OTG_HS;
extern void uvc_process_thread_entry(ULONG arg);
void printf_uvc_descriptor(UX_HOST_CLASS_VIDEO *video_class);
/* USER CODE END PFP */
/**
 * @brief  Application USBX Host Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT MX_USBX_Host_Init(VOID *memory_ptr) {
  UINT ret = UX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*) memory_ptr;

  /* USER CODE BEGIN MX_USBX_Host_MEM_POOL */
  // (void)byte_pool;
  CHAR *memory_pointer;

  /* Allocate the stack for thread 0.  */
  if (tx_byte_allocate(byte_pool, (VOID**) &memory_pointer, USBX_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS) {
    return TX_POOL_ERROR;
  }
  /* USER CODE END MX_USBX_Host_MEM_POOL */

  /* USER CODE BEGIN MX_USBX_Host_Init */
  /* Initialize USBX system. */
  UINT status = ux_system_initialize(memory_pointer, USBX_MEMORY_SIZE, UX_NULL, 0);
  if (status != UX_SUCCESS)
    Error_Handler();

  _ux_utility_error_callback_register(ux_host_error_callback);

  /* Allocate Memory for the ux_app_MsgQueue  */
  if (tx_byte_allocate(byte_pool, (VOID**) &memory_pointer, APP_QUEUE_SIZE * sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS) {
    return TX_POOL_ERROR;
  }

  /* Create the MsgQueue   */
  if (tx_queue_create(&ux_app_MsgQueue, "Message Queue app", TX_1_ULONG, memory_pointer, APP_QUEUE_SIZE * sizeof(ULONG)) != TX_SUCCESS) {
    return TX_QUEUE_ERROR;
  }

  if (tx_byte_allocate(byte_pool, (VOID**) &memory_pointer, (USBX_APP_STACK_SIZE * 2), TX_NO_WAIT) != TX_SUCCESS) {
    return TX_POOL_ERROR;
  }

  if (tx_thread_create(&ux_app_thread, "usbx_app_thread", usbx_app_thread_entry, 0, memory_pointer, USBX_APP_STACK_SIZE, 25, 25, 0, TX_AUTO_START) !=
  TX_SUCCESS) {
    return TX_THREAD_ERROR;
  }
  if (tx_byte_allocate(byte_pool, (VOID**) &memory_pointer, (USBX_APP_STACK_SIZE * 2), TX_NO_WAIT) != TX_SUCCESS) {
    return TX_POOL_ERROR;
  }

  /* Create the storage applicative process thread.  */
  if (tx_thread_create(&uvc_app_thread, "uvc_app_thread", uvc_process_thread_entry, 0, memory_pointer, (USBX_APP_STACK_SIZE * 2), 30, 30, 0, TX_AUTO_START) !=
  TX_SUCCESS) {
    return TX_THREAD_ERROR;
  }
  /* USER CODE END MX_USBX_Host_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
/* video data received callback function. */
static VOID video_transfer_done(UX_TRANSFER *transfer_request) {
  UINT status;
  printf("video transfer done.\r\n");
  status = tx_semaphore_put(&data_received_semaphore);
  if (status != UX_SUCCESS)
    Error_Handler();
}
static VOID video_transfer_done2(UX_TRANSFER *transfer_request) {
  UINT status;
  printf("video transfer done2.\r\n");
  printf("status:%lu\r\n", transfer_request->ux_transfer_request_status);
  printf("type:%u\r\n", transfer_request->ux_transfer_request_type);
  printf("value:%u\r\n", transfer_request->ux_transfer_request_value);
  printf("code:%u\r\n", transfer_request->ux_transfer_request_completion_code);

  // status = tx_semaphore_put(&data_received_semaphore);
  // if (status != UX_SUCCESS)
  //   Error_Handler();
}
/**
 * @brief  Application_thread_entry .
 * @param  ULONG arg
 * @retval Void
 */
void usbx_app_thread_entry(ULONG arg) {
  /* Initialize USBX_Host */
  MX_USB_Host_Init();

  /* Start Application */
  USBH_UsrLog(" **** USB OTG FS Video Host **** \n");
  USBH_UsrLog("USB Host library started.\n");

  /* Initialize Application and Video process */
  USBH_UsrLog("Starting Video Application");
  USBH_UsrLog("Connect your Video Device\n");

  while (1) {
    /* wait for message queue from callback event */
    if (tx_queue_receive(&ux_app_MsgQueue, &ux_dev_info, TX_WAIT_FOREVER) != TX_SUCCESS) {
      Error_Handler();
    }

    if (ux_dev_info.Dev_state == Device_connected) {
      switch (ux_dev_info.Device_Type) {
        case UVC_Device:
          /* Device_information */
          USBH_UsrLog("USB Mass video_class Device Found")
          ;
          USBH_UsrLog("PID: %#x ", (UINT ) video_class->ux_host_class_video_device->ux_device_descriptor.idProduct)
          ;
          USBH_UsrLog("VID: %#x ", (UINT ) video_class->ux_host_class_video_device->ux_device_descriptor.idVendor)
          ;

          /* start File operations */
          USBH_UsrLog("\n*** Start video operations ***\n")
          ;
          /* send queue to uvc_app_process*/
          /* Set transfer callback (do before start transfer). */
          ux_host_class_video_transfer_callback_set(video_class, video_transfer_done);
          video_class->ux_host_class_video_device->ux_device_control_endpoint.ux_endpoint_transfer_request.ux_transfer_request_completion_function =
              video_transfer_done2;
          UINT status = ux_host_class_video_frame_parameters_set(video_class, UX_HOST_CLASS_VIDEO_VS_FORMAT_MJPEG, 640, 480, 400000);
          if (status != UX_SUCCESS)
            Error_Handler();
          status = ux_host_class_video_start(video_class);
          if (status != UX_SUCCESS)
            Error_Handler();
            status = ux_host_class_video_transfer_buffer_add(video_class, video_buffer);
          if (status != UX_SUCCESS)
            Error_Handler();

#if HIGH_BANDWIDTH_EHCI /* Driver HCD must support adding requests list.  */
          /* Build buffer list.  */
          for (i = 0; i < VIDEO_BUFFER_NB; i++)
            video_buffers[i] = video_buffer[i];

          /* Issue transfer request list to start streaming.  */
          status = ux_host_class_video_transfer_buffers_add(video, video_buffers, VIDEO_BUFFER_NB);
          if (status != UX_SUCCESS)
            error_handler();
#elif NORMAL_BANDWIDTH_OHCI /* Driver adds request one by one.  */
          /* Allocate space for video buffer. */
          for (buffer_index = 0; buffer_index < VIDEO_BUFFER_NB; buffer_index++) {
            /* Add buffer to the video device for video streaming data. */
            status = ux_host_class_video_transfer_buffer_add(video, video_buffer[buffer_index]);
            if (status != UX_SUCCESS)
              error_handler();
          }
#endif

          break;

        case Unknown_Device:
          USBH_ErrLog("!! Unsupported UVC_Device plugged !!")
          ;
          ux_dev_info.Dev_state = No_Device;
          break;

        case Unsupported_Device:
          USBH_ErrLog("!! Unabble to start Device !!")
          ;
          break;

        default:
          break;
      }
    } else {
      /*clear video_class instance*/
      tx_thread_sleep(MS_TO_TICK(50));
    }
  }
}

/**
 * @brief MX_USB_Host_Init
 *        Initialization of USB Host.
 * Init USB Host Library, add supported class and start the library
 * @retval None
 */
UINT MX_USB_Host_Init(void) {
  UINT ret = UX_SUCCESS;
  /* USER CODE BEGIN USB_Host_Init_PreTreatment_0 */
  /* USER CODE END USB_Host_Init_PreTreatment_0 */

  /* The code below is required for installing the host portion of USBX.  */
  if (ux_host_stack_initialize(ux_host_event_callback) != UX_SUCCESS) {
    return UX_ERROR;
  }

  /* Register video_class class. */
  if (ux_host_stack_class_register(_ux_system_host_class_video_name, _ux_host_class_video_entry) != UX_SUCCESS) {
    return UX_ERROR;
  }
  UINT status = tx_semaphore_create(&data_received_semaphore, "payload semaphore", 0);
  if (status != UX_SUCCESS)
    Error_Handler();
  /* Initialize the LL driver */
  MX_USB_OTG_HS_HCD_Init();

  /* Register all the USB host controllers available in this system. */
  if (ux_host_stack_hcd_register(_ux_system_host_hcd_stm32_name, ux_hcd_stm32_initialize, USB_OTG_HS_PERIPH_BASE, (ULONG) &hhcd_USB_OTG_HS) != UX_SUCCESS) {
    return UX_ERROR;
  }

  /* Drive vbus to be addedhere */
  // USBH_DriverVBUS(USB_VBUS_TRUE);
  /* Enable USB Global Interrupt */
  HAL_HCD_Start(&hhcd_USB_OTG_HS);

  /* USER CODE BEGIN USB_Host_Init_PreTreatment_1 */
  /* USER CODE END USB_Host_Init_PreTreatment_1 */

  /* USER CODE BEGIN USB_Host_Init_PostTreatment */
  /* USER CODE END USB_Host_Init_PostTreatment */
  return ret;
}

/**
 * @brief ux_host_event_callback
 * @param ULONG event
 This parameter can be one of the these values:
 1 : UX_DEVICE_INSERTION
 2 : UX_DEVICE_REMOVAL
 UX_HOST_CLASS * Current_class
 VOID * Current_instance
 * @retval Status
 */
UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *Current_class, VOID *Current_instance) {
  UINT status;
  UX_HOST_CLASS *host_class;
  switch (event) {
    case UX_DEVICE_INSERTION:
      /* Get current Hid Class */
      status = ux_host_stack_class_get(_ux_system_host_class_video_name, &host_class);
      if (status == UX_SUCCESS) {
        if ((host_class == Current_class) && (video_class == NULL)) {
          printf("ux_host event callback:%lu\r\n", event);
          /* get current uvc Instance */
          video_class = Current_instance;

          if (video_class == NULL) {
            USBH_UsrLog("unable to start media ");
            ux_dev_info.Device_Type = Unsupported_Device;
            ux_dev_info.Dev_state = Device_connected;
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          } else {
          }

          if (video_class->ux_host_class_video_state != (ULONG) UX_HOST_CLASS_INSTANCE_LIVE) {
            ux_dev_info.Device_Type = Unsupported_Device;
            ux_dev_info.Dev_state = Device_connected;
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          } else {
            /* USB _Video_ Device found */
            USBH_UsrLog("USB Device Plugged");

            /* update USB device Type */
            ux_dev_info.Device_Type = UVC_Device;
            ux_dev_info.Dev_state = Device_connected;

            /* put a message queue to usbx_app_thread_entry */
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }
        }
      } else {
        /* No Video class found */
        USBH_ErrLog("NO Video Class found");
      }
      break;

    case UX_DEVICE_REMOVAL:

      if (Current_instance == video_class) {
        /* free Instance */
        video_class = NULL;
        USBH_UsrLog("USB Device Unplugged");
        ux_dev_info.Dev_state = No_Device;
        ux_dev_info.Device_Type = Unknown_Device;
        tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
      }
      break;

    default:
      break;
  }

  return (UINT) UX_SUCCESS;
}

/**
 * @brief ux_host_error_callback
 * @param ULONG event
 UINT system_context
 UINT error_code
 * @retval Status
 */
VOID ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code) {
  switch (error_code) {
    case UX_DEVICE_ENUMERATION_FAILURE:
      ux_dev_info.Device_Type = Unknown_Device;
      ux_dev_info.Dev_state = Device_connected;
      tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
      break;

    case UX_NO_DEVICE_CONNECTED:
      USBH_UsrLog("USB Device disconnected")
      ;
      break;
    default:
      USBH_UsrLog("USB Device host error_code:%d", error_code)
      ;
      break;
  }
}

/**
 * @brief  Drive VBUS.
 * @param  state : VBUS state
 *          This parameter can be one of the these values:
 *           1 : VBUS Active
 *           0 : VBUS Inactive
 * @retval Status
 */
void USBH_DriverVBUS(uint8_t state) {
  /* USER CODE BEGIN 0 */

  /* USER CODE END 0*/

  if (state == USB_VBUS_TRUE) {
    /* Drive high Charge pump */
    /* Add IOE driver control */
    /* USER CODE BEGIN DRIVE_HIGH_CHARGE_FOR_HS */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
    /* USER CODE END DRIVE_HIGH_CHARGE_FOR_HS */
  } else {
    /* Drive low Charge pump */
    /* Add IOE driver control */
    /* USER CODE BEGIN DRIVE_LOW_CHARGE_FOR_HS */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
    /* USER CODE END DRIVE_LOW_CHARGE_FOR_HS */
  }

  HAL_Delay(200);
}
void printf_uvc_descriptor(UX_HOST_CLASS_VIDEO *video_class) {
  printf("Device Descriptor: ");
  printf(" bLength %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.bLength);
  printf(" bDescriptorType %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.bDescriptorType);
  printf(" bcdUSB %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.bcdUSB);
  printf(" bDeviceClass %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.bDeviceClass);
  printf(" bDeviceSubClass %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.bDeviceSubClass);
  printf(" bDeviceProtocol %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.bDeviceProtocol);
  printf(" bMaxPacketSize0 %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.bMaxPacketSize0);
  printf(" idVendor 0x%lx\n", video_class->ux_host_class_video_device->ux_device_descriptor.idVendor);
  printf(" idProduct 0x%lx\n", video_class->ux_host_class_video_device->ux_device_descriptor.idProduct);
  printf(" bcdDevice %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.bcdDevice);
  printf(" iManufacturer %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.iManufacturer);
  printf(" iProduct %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.iProduct);
  printf(" iSerialNumber %lu\n", video_class->ux_host_class_video_device->ux_device_descriptor.iSerialNumber);
  UX_INTERFACE *interface = video_class->ux_host_class_video_device->ux_device_current_configuration->ux_configuration_first_interface;
  printf(" Configuration Descriptor: ");
  printf("  bLength %lu\n", video_class->ux_host_class_video_device->ux_device_current_configuration->ux_configuration_descriptor.bLength);
  printf("  bDescriptorType %lu\n", video_class->ux_host_class_video_device->ux_device_current_configuration->ux_configuration_descriptor.bDescriptorType);
  printf("  wTotalLength 0x%lx\n", video_class->ux_host_class_video_device->ux_device_current_configuration->ux_configuration_descriptor.wTotalLength);
  printf("  bNumInterfaces %lu\n", video_class->ux_host_class_video_device->ux_device_current_configuration->ux_configuration_descriptor.bNumInterfaces);
  printf("  bConfigurationValue %lu\n",
         video_class->ux_host_class_video_device->ux_device_current_configuration->ux_configuration_descriptor.bConfigurationValue);
  printf("  iConfiguration %lu\n", video_class->ux_host_class_video_device->ux_device_current_configuration->ux_configuration_descriptor.iConfiguration);
  printf("  bmAttributes 0x%lx\n", video_class->ux_host_class_video_device->ux_device_current_configuration->ux_configuration_descriptor.bmAttributes);
  printf("  bDescriptorType %luMA\n", video_class->ux_host_class_video_device->ux_device_current_configuration->ux_configuration_descriptor.MaxPower);
  while (interface != NULL) {
    if (interface->ux_interface_next_interface != NULL) {
      printf(" Interface Descriptor: ");
      printf("  bLength %lu\n", interface->ux_interface_descriptor.bLength);
      printf("  bDescriptorType %lu\n", interface->ux_interface_descriptor.bDescriptorType);
      printf("  bInterfaceNumber %lu\n", interface->ux_interface_descriptor.bInterfaceNumber);
      printf("  bNumEndpoints %lu\n", interface->ux_interface_descriptor.bNumEndpoints);
      printf("  bInterfaceClass %lu\n", interface->ux_interface_descriptor.bInterfaceClass);
      printf("  bInterfaceSubClass %lu\n", interface->ux_interface_descriptor.bInterfaceSubClass);
      printf("  bInterfaceProtocol %lu\n", interface->ux_interface_descriptor.bInterfaceProtocol);
      printf("  iInterface %lu\n", interface->ux_interface_descriptor.iInterface);
      interface = interface->ux_interface_next_interface;
    }
  }
}

/* USER CODE END 1 */
