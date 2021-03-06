    include bl_config.inc

SYSCTL_RESC                     equ     0x400fe05c
SYSCTL_RESC_MOSCFAIL            equ     0x00010000
NVIC_VTABLE                     equ     0xe000ed08

    export  IntDefaultHandler
    export  Reset_Handler_In_SRAM 
    export  UpdateHandler_In_SRAM

;******************************************************************************
;
; Put the assembler into the correct configuration.
;
;******************************************************************************
    thumb
    require8
    preserve8

;******************************************************************************
;
; This portion of the file goes into the text section.
; 下边两句不定义 程序在ROM中执行
; 定义后程序起始地址在RAM中
;******************************************************************************
    align   4
    area    ||.text||, code, readonly, align=2

Reset_Handler_In_SRAM
    ;
    ; Call the user-supplied low level hardware initialization function
    ; if provided.
    ;
    if      :def:_BL_HW_INIT_FN_HOOK
    import  $_BL_HW_INIT_FN_HOOK
    bl      $_BL_HW_INIT_FN_HOOK
    endif

    ;
    ; See if an update should be performed.
    ;
    import  CheckForceUpdate
    bl      CheckForceUpdate            ;检查强制升级状态
    cbz     r0, CallApplication         ;返回不等于0 进入升级程序(状态)
                                        ;否则进入用户程序
    ;
    ; Configure the microcontroller.
    ;
EnterBootLoader                         ;进入升级程序
    if      :def:_ENET_ENABLE_UPDATE    ;是否通过网络升级(编译选项)
    import  ConfigureEnet
    bl      ConfigureEnet               ;初始化网络
    elif    :def:_CAN_ENABLE_UPDATE     ;是否通过CAN升级
    import  ConfigureCAN
    bl      ConfigureCAN                ;初始化CAN接口
    elif    :def:_USB_ENABLE_UPDATE     ;是否通过USB升级
    import  ConfigureUSB
    bl      ConfigureUSB                ;初始化USB
    else                                ;其他升级方式 UART I2C SSI 
    import  ConfigureDevice
    bl      ConfigureDevice             ;初始化设备
    endif

    ;
    ; Call the user-supplied initialization function if provided.
    ;
    if      :def:_BL_INIT_FN_HOOK
    import  $_BL_INIT_FN_HOOK
    bl      $_BL_INIT_FN_HOOK
    endif

    ;
    ; Branch to the update handler.
    ;
    if      :def:_ENET_ENABLE_UPDATE
    import  UpdateBOOTP
    b       UpdateBOOTP                   ;跳转到网络升级程序
    elif    :def:_CAN_ENABLE_UPDATE
    import  UpdaterCAN
    b       UpdaterCAN                    ;跳转到CAN总线升级程序
    elif    :def:_USB_ENABLE_UPDATE
    import  UpdaterUSB
    b       UpdaterUSB
    else
    import  Updater
    b       Updater                       ;跳转到其他升级程序
    endif

    ;
    ; This is a second symbol to allow starting the application from the boot
    ; loader the linker may not like the perceived jump.
    ;
    export StartApplication
StartApplication                          ;进入用户程序
    ;
    ; Call the application via the reset handler in its vector table.  Load the
    ; address of the application vector table.
    ;
CallApplication
    ;
    ; Copy the application's vector table to the target address if necessary.
    ; Note that incorrect boot loader configuration could cause this to
    ; corrupt the code!  Setting VTABLE_START_ADDRESS to 0x20000000 (the start
    ; of SRAM) is safe since this will use the same memory that the boot loader
    ; already uses for its vector table.  Great care will have to be taken if
    ; other addresses are to be used.
    ;
    if (_APP_START_ADDRESS != _VTABLE_START_ADDRESS)  ;用户程序地址是否等于中断向量地址
    movw    r0, #(_VTABLE_START_ADDRESS & 0xffff)     ;不相等执行 拷贝中断向量表地址->r0 用户程序地址->r1
    if (_VTABLE_START_ADDRESS > 0xffff)
    movt    r0, #(_VTABLE_START_ADDRESS >> 16)
    endif
    movw    r1, #(_APP_START_ADDRESS & 0xffff)
    if (_APP_START_ADDRESS > 0xffff)
    movt    r1, #(_APP_START_ADDRESS >> 16)
    endif

    ;
    ; Calculate the end address of the vector table assuming that it has the
    ; maximum possible number of vectors.  We don't know how many the app has
    ; populated so this is the safest approach though it may copy some non
    ; vector data if the app table is smaller than the maximum.
    ;
    movw    r2, #(70 * 4)
    adds    r2, r2, r0
VectorCopyLoop
        ldr     r3, [r1], #4
        str     r3, [r0], #4
        cmp     r0, r2                ;拷贝用户程序中断向量表 到RAM
        blt     VectorCopyLoop
    endif     ;if (_APP_START_ADDRESS != _VTABLE_START_ADDRESS)

    ;
    ; Set the vector table address to the beginning of the application.
    ;
    movw    r0, #(_VTABLE_START_ADDRESS & 0xffff)
    if (_VTABLE_START_ADDRESS > 0xffff)
    movt    r0, #(_VTABLE_START_ADDRESS >> 16)
    endif
    movw    r1, #(NVIC_VTABLE & 0xffff)
    movt    r1, #(NVIC_VTABLE >> 16)
    str     r0, [r1]

    ;
    ; Load the stack pointer from the application's vector table.
    ;
    if (_APP_START_ADDRESS != _VTABLE_START_ADDRESS)
    movw    r0, #(_APP_START_ADDRESS & 0xffff)
    if (_APP_START_ADDRESS > 0xffff)
    movt    r0, #(_APP_START_ADDRESS >> 16)
    endif
    endif
    ldr     sp, [r0]

    ;
    ; Load the initial PC from the application's vector table and branch to
    ; the application's entry point.
    ;
    ldr     r0, [r0, #4]        ;拷贝用户程序入口 Reset_Handler
    bx      r0                  ;跳转到用户程序执行

;******************************************************************************
;
; The update handler, which gets called when the application would like to
; start an update.
;
;******************************************************************************
UpdateHandler_In_SRAM
    ;
    ; Load the stack pointer from the vector table.
    ;
    movs    r0, #0x0000
    ldr     sp, [r0]

    ;
    ; Call the user-supplied low level hardware initialization function
    ; if provided.
    ;
    if      :def:_BL_HW_INIT_FN_HOOK
    bl      $_BL_HW_INIT_FN_HOOK
    endif

    ;
    ; Call the user-supplied re-initialization function if provided.
    ;
    if      :def:_BL_REINIT_FN_HOOK
    import  $_BL_REINIT_FN_HOOK
    bl      $_BL_REINIT_FN_HOOK
    endif

    ;
    ; Branch to the update handler.
    ;
    if      :def:_ENET_ENABLE_UPDATE
    b       UpdateBOOTP
    elif    :def:_CAN_ENABLE_UPDATE
    import  AppUpdaterCAN
    b       AppUpdaterCAN
    elif    :def:_USB_ENABLE_UPDATE
    import  AppUpdaterUSB
    b       AppUpdaterUSB
    else
    b       Updater
    endif

;******************************************************************************
;
; The NMI handler.
;
;******************************************************************************
    if      :def:_ENABLE_MOSCFAIL_HANDLER
NmiSR_In_SRAM
    ;
    ; Restore the stack frame.
    ;
    mov     lr, r12
    stm     sp, {r4-r11}

    ;
    ; Save the link register.
    ;
    mov     r9, lr

    ;
    ; Call the user-supplied low level hardware initialization function
    ; if provided.
    ;
    if      :def:_BL_HW_INIT_FN_HOOK
    bl      _BL_HW_INIT_FN_HOOK
    endif

    ;
    ; See if an update should be performed.
    ;
    bl      CheckForceUpdate
    cbz     r0, EnterApplication

        ;
        ; Clear the MOSCFAIL bit in RESC.
        ;
        movw    r0, #(SYSCTL_RESC & 0xffff)
        movt    r0, #(SYSCTL_RESC >> 16)
        ldr     r1, [r0]
        bic     r1, r1, #SYSCTL_RESC_MOSCFAIL
        str     r1, [r0]

        ;
        ; Fix up the PC on the stack so that the boot pin check is bypassed
        ; (since it has already been performed).
        ;
        ldr     r0, =EnterBootLoader
        bic     r0, #0x00000001
        str     r0, [sp, #0x18]
        
        ;
        ; Return from the NMI handler.  This will then start execution of the
        ; boot loader.
        ;
        bx      r9

    ;
    ; Restore the link register.
    ;
EnterApplication
    mov     lr, r9

    ;
    ; Copy the application's vector table to the target address if necessary.
    ; Note that incorrect boot loader configuration could cause this to
    ; corrupt the code!  Setting VTABLE_START_ADDRESS to 0x20000000 (the start
    ; of SRAM) is safe since this will use the same memory that the boot loader
    ; already uses for its vector table.  Great care will have to be taken if
    ; other addresses are to be used.
    ;
    if (_APP_START_ADDRESS != _VTABLE_START_ADDRESS)
    movw    r0, #(_VTABLE_START_ADDRESS & 0xffff)
    if (_VTABLE_START_ADDRESS > 0xffff)
    movt    r0, #(_VTABLE_START_ADDRESS >> 16)
    endif
    movw    r1, #(_APP_START_ADDRESS & 0xffff)
    if (_APP_START_ADDRESS > 0xffff)
    movt    r1, #(_APP_START_ADDRESS >> 16)
    endif

    ;
    ; Calculate the end address of the vector table assuming that it has the
    ; maximum possible number of vectors.  We don't know how many the app has
    ; populated so this is the safest approach though it may copy some non
    ; vector data if the app table is smaller than the maximum.
    ;
    movw    r2, #(70 * 4)
    adds    r2, r2, r0
VectorCopyLoop2
        ldr     r3, [r1], #4
        str     r3, [r0], #4
        cmp     r0, r2
        blt     VectorCopyLoop2
    endif

    ;
    ; Set the application's vector table start address.  Typically this is the
    ; application start address but in some cases an application may relocate
    ; this so we can't assume that these two addresses are equal.
    ;
    movw    r0, #(_VTABLE_START_ADDRESS & 0xffff)
    if (_VTABLE_START_ADDRESS > 0xffff)
    movt    r0, #(_VTABLE_START_ADDRESS >> 16)
    endif
    movw    r1, #(NVIC_VTABLE & 0xffff)
    movt    r1, #(NVIC_VTABLE >> 16)
    str     r0, [r1]

    ;
    ; Remove the NMI stack frame from the boot loader's stack.
    ;
    ldmia   sp, {r4-r11}

    ;
    ; Get the application's stack pointer.
    ;
    if (_APP_START_ADDRESS != _VTABLE_START_ADDRESS)
    movw    r0, #(_APP_START_ADDRESS & 0xffff)
    if (_APP_START_ADDRESS > 0xffff)
    movt    r0, #(_APP_START_ADDRESS >> 16)
    endif
    endif
    ldr     sp, [r0, #0x00]

    ;
    ; Fix up the NMI stack frame's return address to be the reset handler of
    ; the application.
    ;
    ldr     r10, [r0, #0x04]
    bic     r10, #0x00000001

    ;
    ; Store the NMI stack frame onto the application's stack.
    ;
    stmdb   sp!, {r4-r11}

    ;
    ; Branch to the application's NMI handler.
    ;
    ldr     r0, [r0, #0x08]
    bx      r0
    endif

;******************************************************************************
;
; The default interrupt handler.
;
;******************************************************************************
IntDefaultHandler
    ;
    ; Loop forever since there is nothing that we can do about an unexpected
    ; interrupt.
    ;
    b       .

;******************************************************************************
;
; Provides a small delay.  The loop below takes 3 cycles/loop.
;
;******************************************************************************
    export  Delay
Delay
    subs    r0, #1
    bne     Delay
    bx      lr

;******************************************************************************
;
; This is the end of the file.
;
;******************************************************************************
    align   4
    end
