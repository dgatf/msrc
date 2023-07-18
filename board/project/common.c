#include "common.h"

float get_average(float alpha, float prev_value, float new_value)
{
    if (isnan(new_value))
        return prev_value;
    if (alpha > 1)
        return new_value;
    else
        return (1 - alpha) * prev_value + alpha * new_value;
}

float get_consumption(float current, uint16_t current_max, uint32_t *timestamp)
{
    if (!*timestamp)
    {
        *timestamp = time_us_32();
        return 0;
    }
    uint32_t now = time_us_32();                   // us
    uint32_t interval = (now - *timestamp) / 1000; // ms
    float mah = current * interval / 3600.0;
    *timestamp = now;
    if (interval > 2000 || (current_max && (mah > current_max * interval / 3600.0)))
        return 0;
    return mah;
}

float voltage_read(uint8_t adc_num)
{
    adc_select_input(adc_num);
    return adc_read() * BOARD_VCC / ADC_RESOLUTION;
}

float get_altitude(float pressure, float temperature, float P0)
{
    if (P0 == 0)
        return 0;
    return (temperature + 273.15) * (1000 / 6.5) * (1 - pow(pressure / P0, 1 / 5.256));
}


/*
void circular_buffer_add(buffer_node_t *node, void *item)
{
    buffer_node_t *new_node = malloc(sizeof(buffer_node_t));
    new_node->item = item;
    if (node == NULL)
    {
        node = new_node;
        new_node->next = new_node;
    }
    else
    {
        new_node->next = node->next;
        node->next = new_node;
        node = new_node;
    }
}

void *circular_buffer_current()
{
    if (node)
        return node->item;
    return NULL;
}

void circular_buffer_next(buffer_node_t *node)
{
    if (node)
        node = node->next;
}

void circular_buffer_empty(buffer_node_t *node)
{
    if (node)
    {
        buffer_node_t *first_node, *next_node;
        first_node = node;
        do
        {
            next_node = node->next;
            free(node->item);
            free(node);
            node = next_node;
        } while (next_node != first_node);
        node = NULL;
    }
}
*/