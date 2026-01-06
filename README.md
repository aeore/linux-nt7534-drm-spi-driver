# linux-nt7534-drm-spi-driver
This is Novatek NT7534 linux driver for Linux Tiny DRM subsystem

It supports SPI way of communication with display only. Parallel bus is not supported.
Tested on Linux kernel 6.6.
Display backlight is also supported. Example dts to use with this driver:
```
...
        backlight_lcd: backlight {
		compatible = "gpio-backlight";
		gpios = <&portb 8 GPIO_ACTIVE_HIGH>;
		default-brightness-level = <1>;
		default-on;
	};
...

&spi0 {
    status = "okay";
    /delete-node/ spidev@0;

    display@0 {
        compatible = "novatek,nt7534";
        reg = <0>;
        spi-max-frequency = <10000000>;

        reset-gpios = <&portb 15 GPIO_ACTIVE_LOW>;
        a0-gpios    = <&portb 12 GPIO_ACTIVE_HIGH>;
        backlight = <&backlight_lcd>;
    };
};
```

Also you can use rotation key to rotate screen of your display

## License
This driver is licensed under terms and conditions of GPLv2. See LICENCE for detailed information

## Contact
Yuriy Gurin <ygurin@outlook.com>
