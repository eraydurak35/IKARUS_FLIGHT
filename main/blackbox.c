#include "blackbox.h"
#include <string.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"


static FILE *flight_file = NULL;
static FILE *nav_file = NULL;
static FILE *index_file = NULL;
static int index_number = 0;

static const char *index_file_path = MOUNT_POINT"/INDEX.TXT";
static char flight_file_path[25];
static char nav_file_path[25];
static char path_start[] = MOUNT_POINT"/";
static char path_flight_end[] = "/flt.bin";
static char path_nav_end[] = "/nav.bin";

static void read_file_index_num();
static void save_file_index_num();


uint8_t blackbox_init()
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = 
    {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;

    spi_bus_config_t bus_cfg = 
    {
        .mosi_io_num = BB_MOSI,
        .miso_io_num = BB_MISO,
        .sclk_io_num = BB_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = BB_CS;
    slot_config.host_id = host.slot;

    esp_err_t ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) 
    {
        return 0;
    }
    read_file_index_num();
    return 1;
}


static void read_file_index_num()
{
    index_file = fopen(index_file_path, "r+");
    if (index_file != NULL)
    {
        fscanf(index_file, "%d", &index_number);
        fclose(index_file);
    }
}

static void save_file_index_num()
{
    index_file = fopen(index_file_path, "r+");
    if (index_file != NULL)
    {
        fprintf(index_file, "%d", index_number + 1);
        fclose(index_file);
    }
}

uint8_t create_and_open_bin_file()
{
    char index_char[11];
    sprintf(index_char, "%d", index_number);

    strcpy(flight_file_path, path_start);
    strcat(flight_file_path, index_char);
    mkdir(flight_file_path, 0777);
    strcat(flight_file_path, path_flight_end);

    strcpy(nav_file_path, path_start);
    strcat(nav_file_path, index_char);
    mkdir(nav_file_path, 0777);
    strcat(nav_file_path, path_nav_end);

    flight_file = fopen(flight_file_path, "wb");
    nav_file = fopen(nav_file_path, "wb");
    if (flight_file == NULL || nav_file == NULL)
        return 0;
    return 1;
}

void close_bin_file()
{
    if (flight_file != NULL)
    {
        fclose(flight_file);
        flight_file = NULL;
    }

    if (nav_file != NULL)
    {
        fclose(nav_file);
        nav_file = NULL;
    }
    
    save_file_index_num();
}

void write_flight_to_bin_file(uint8_t *data, uint8_t size)
{
    if (flight_file != NULL)
    {
        fwrite(data, size, 1, flight_file);
    }
}

void write_navigation_to_bin_file(uint8_t *data, uint8_t size)
{
    if (nav_file != NULL)
    {
        fwrite(data, size, 1, nav_file);
    }
}


