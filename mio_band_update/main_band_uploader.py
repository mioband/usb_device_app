from stm32loader import main as loader
import esptool
import serial


class Band_uploader:
    def __init__(self):
        self.esp_file = './firmwares/firmware.bin'
        self.stm_file = './firmwares/stm_firmware_v00.bin'
        self.esp_uploader_file = './config/esp_firmware_uploader.bin'
        self.new_stm_upload = True
        self.new_esp_upload = True

    def stm_upload(self, port, new_stm_file):
        loader.main('-p', port, '-e', '-w', '-v', new_stm_file)

    def esp_upload(self, port, new_esp_file):
        esptool.main(
            ['-p', port, '-b', '115200', '--before', 'default_reset', '--after', 'hard_reset', '--chip', 'esp32',
             'write_flash', '-z', '--flash_mode', 'dio', '--flash_size', 'detect', '--flash_freq', '40m', '0x1000',
             './config/bootloader_dio_40m.bin', '0x8000', './config/partitions.bin', '0xe000', './config/boot_app0.bin',
             '0x10000', new_esp_file])

    def check_update(self, new_stm, new_esp):
        self.new_stm_upload = new_stm
        self.new_esp_upload = new_esp

    def band_upload(self, port, new_stm_file, new_esp_file):
        if self.new_esp_upload and self.new_esp_upload:
            self.esp_upload(port, self.esp_uploader_file)
            self.stm_upload(port, new_stm_file)
            self.esp_upload(port, new_esp_file)
            self.esp_file = new_esp_file
            self.stm_file = new_stm_file
        elif self.new_esp_upload:
            self.esp_upload(port, new_esp_file)
            self.esp_file = new_esp_file
        elif self.new_stm_upload:
            self.esp_upload(port, self.esp_uploader_file)
            self.stm_upload(port, new_stm_file)
            self.esp_upload(port, self.esp_file)
            self.stm_file = new_stm_file

    def band_reload(self, port):
        self.band_upload(port, self.stm_file, self.esp_file)


class ConnectorUploader(Band_uploader):
    def esp_upload(self, port, new_esp_file):
        try:
            connection = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                       stopbits=serial.STOPBITS_ONE, timeout=1, write_timeout=20)
            connection.write(bytearray('FA\n', encoding='utf-8'))
            connection.close()
        except serial.SerialException as err:
            print(err)

        esptool.main(
            ['-p', port, '-b', '115200', '--before', 'default_reset', '--after', 'hard_reset', '--chip', 'esp32',
             'write_flash', '-z', '--flash_mode', 'dio', '--flash_size', 'detect', '--flash_freq', '40m', '0x1000',
             './config/bootloader_dio_40m.bin', '0x8000', './config/partitions.bin', '0xe000', './config/boot_app0.bin',
             '0x10000', new_esp_file])


if __name__ == '__main__':
    band_up = ConnectorUploader()
    band_up.check_update(False, True)
    band_up.esp_upload('COM12', './firmwares/firmware.bin')  # 'COM12' - case of the port name and number, they can be different
