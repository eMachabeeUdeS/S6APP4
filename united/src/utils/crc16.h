/**
 * @file crc16.h
 * @author Étienne Machabée (mace2801) et Coralie Grégoire (grec3306)
 * @brief Fonctions pour calculer le CRC16 d'un tableau de données
 * @version 0.1
 * @date 2022-06-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef CRC16_H   /* Include guard */
#define CRC16_H

#include "Particle.h"

/**
 * @brief Effectue un CRC16 sur des données
 * 
 * @param data Tableau de données en uint8_t
 * @param size Longueur du tableau en uint16_t
 * @return uint16_t Résultat du CRC16
 */
uint16_t gen_crc16(const uint8_t *data, uint16_t size);

#endif // CRC16_H