const uint16_t P400[] = { PIN_PWM | CHANNEL_6 | PWM_CHANNEL_A | GPT_ODD_CFG, PIN_SCL | CHANNEL_0, PIN_INTERRUPT | CHANNEL_0,
                          SCI_CHANNEL | PIN_SCK | CHANNEL_0 | SCI_EVEN_CFG, SCI_CHANNEL | PIN_SCK | CHANNEL_1 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P401[] = { PIN_PWM | CHANNEL_6 | PWM_CHANNEL_B | GPT_ODD_CFG, PIN_SDA | CHANNEL_0, PIN_INTERRUPT | CHANNEL_5,
                          SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_0 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_1 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P402[] = { PIN_INTERRUPT | CHANNEL_4, SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_1 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P213[] = { PIN_PWM | CHANNEL_0 | PWM_CHANNEL_A | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_2,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_1 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P212[] = { PIN_PWM | CHANNEL_0 | PWM_CHANNEL_B | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_3,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_1 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P411[] = { PIN_PWM | CHANNEL_6 | PWM_CHANNEL_A | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_4,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_0 | SCI_EVEN_CFG, PIN_MOSI | CHANNEL_0 | LAST_ITEM_GUARD };

const uint16_t P410[] = { PIN_PWM | CHANNEL_6 | PWM_CHANNEL_B | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_5,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_0 | SCI_EVEN_CFG, PIN_MISO | CHANNEL_0 | LAST_ITEM_GUARD };

const uint16_t P409[] = { PIN_PWM | CHANNEL_5 | PWM_CHANNEL_A | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_6,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_9 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P408[] = { PIN_PWM | CHANNEL_5 | PWM_CHANNEL_B | GPT_ODD_CFG, PIN_SCL | CHANNEL_0, PIN_INTERRUPT | CHANNEL_7,
                          SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_1 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_9 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P407[] = { PIN_SDA | CHANNEL_0, SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_0 | SCI_EVEN_CFG | LAST_ITEM_GUARD };

const uint16_t P206[] = { PIN_SDA | CHANNEL_1, PIN_INTERRUPT | CHANNEL_0,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_0 | SCI_EVEN_CFG | LAST_ITEM_GUARD };

const uint16_t P205[] = { PIN_PWM | CHANNEL_4 | PWM_CHANNEL_A | GPT_ODD_CFG, PIN_SCL | CHANNEL_1, PIN_INTERRUPT | CHANNEL_1,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_0 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_9 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P200[] = { PIN_INTERRUPT | CHANNEL_0 | LAST_ITEM_GUARD };

const uint16_t P304[] = { PIN_PWM | CHANNEL_7 | PWM_CHANNEL_A | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_9 | LAST_ITEM_GUARD };

const uint16_t P303[] = { PIN_PWM | CHANNEL_7 | PWM_CHANNEL_B | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P302[] = { PIN_PWM | CHANNEL_4 | PWM_CHANNEL_A | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_5,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_2 | SCI_EVEN_CFG | LAST_ITEM_GUARD };

const uint16_t P301[] = { PIN_PWM | CHANNEL_4 | PWM_CHANNEL_B | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_6,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_2 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_9 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P300[] = { PIN_PWM | CHANNEL_0 | PWM_CHANNEL_A | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P108[] = { PIN_PWM | CHANNEL_0 | PWM_CHANNEL_B | GPT_ODD_CFG,
                          SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_9 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P109[] = { PIN_CAN_TX | CHANNEL_0, PIN_PWM | CHANNEL_1 | PWM_CHANNEL_A | GPT_ODD_CFG, SCI_CHANNEL | PIN_SCK | CHANNEL_1 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_9 | SCI_ODD_CFG, PIN_MOSI | CHANNEL_1 | LAST_ITEM_GUARD };

const uint16_t P110[] = { PIN_CAN_RX | CHANNEL_0,
                          PIN_PWM | CHANNEL_1 | PWM_CHANNEL_B | GPT_ODD_CFG,
                          PIN_INTERRUPT | CHANNEL_3,
                          SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_2 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_9 | SCI_ODD_CFG,
                          PIN_MISO | CHANNEL_1 | LAST_ITEM_GUARD };

const uint16_t P111[] = { PIN_PWM | CHANNEL_3 | PWM_CHANNEL_A | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_4,
                          SCI_CHANNEL | PIN_SCK | CHANNEL_2 | SCI_EVEN_CFG, SCI_CHANNEL | PIN_SCK | CHANNEL_9 | SCI_ODD_CFG,
                          PIN_SCK | CHANNEL_1 | LAST_ITEM_GUARD };

const uint16_t P112[] = { PIN_PWM | CHANNEL_3 | PWM_CHANNEL_B | GPT_ODD_CFG, SCI_CHANNEL | PIN_SCK | CHANNEL_1 | SCI_ODD_CFG,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_2 | SCI_EVEN_CFG | LAST_ITEM_GUARD };

const uint16_t P113[] = { PIN_PWM | CHANNEL_2 | PWM_CHANNEL_A | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P107[] = { PIN_PWM | CHANNEL_0 | PWM_CHANNEL_A | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P106[] = { PIN_PWM | CHANNEL_0 | PWM_CHANNEL_B | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P105[] = { PIN_PWM | CHANNEL_1 | PWM_CHANNEL_A | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_0 | LAST_ITEM_GUARD };

const uint16_t P104[] = { PIN_PWM | CHANNEL_1 | PWM_CHANNEL_B | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_1,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_0 | SCI_EVEN_CFG | LAST_ITEM_GUARD };

const uint16_t P103[] = { PIN_ANALOG | CHANNEL_19, PIN_CAN_TX | CHANNEL_0, PIN_PWM | CHANNEL_2 | PWM_CHANNEL_A | GPT_ODD_CFG,
                          SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_0 | SCI_EVEN_CFG | LAST_ITEM_GUARD };

const uint16_t P102[] = { PIN_ANALOG | CHANNEL_20,
                          PIN_CAN_RX | CHANNEL_0,
                          PIN_PWM | CHANNEL_2 | PWM_CHANNEL_B | GPT_ODD_CFG,
                          SCI_CHANNEL | PIN_SCK | CHANNEL_0 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_2 | SCI_ODD_CFG,
                          PIN_SCK | CHANNEL_0 | LAST_ITEM_GUARD };

const uint16_t P101[] = { PIN_ANALOG | CHANNEL_21,
                          PIN_PWM | CHANNEL_5 | PWM_CHANNEL_A | GPT_ODD_CFG,
                          PIN_SDA | CHANNEL_1,
                          PIN_INTERRUPT | CHANNEL_1,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_0 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_1 | SCI_ODD_CFG,
                          PIN_MOSI | CHANNEL_0 | LAST_ITEM_GUARD };

const uint16_t P100[] = { PIN_ANALOG | CHANNEL_22,
                          PIN_PWM | CHANNEL_5 | PWM_CHANNEL_B | GPT_ODD_CFG,
                          PIN_SCL | CHANNEL_1,
                          PIN_INTERRUPT | CHANNEL_2,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_0 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_SCK | CHANNEL_1 | SCI_ODD_CFG,
                          PIN_MISO | CHANNEL_0 | LAST_ITEM_GUARD };

const uint16_t P500[] = { PIN_ANALOG | CHANNEL_16, PIN_PWM | CHANNEL_2 | PWM_CHANNEL_A | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P501[] = { PIN_ANALOG | CHANNEL_17, PIN_PWM | CHANNEL_2 | PWM_CHANNEL_B | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_11,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_1 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P502[] = { PIN_ANALOG | CHANNEL_18, PIN_PWM | CHANNEL_3 | PWM_CHANNEL_B | GPT_ODD_CFG, PIN_INTERRUPT | CHANNEL_12,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_1 | SCI_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P015[] = { PIN_ANALOG | CHANNEL_10, PIN_INTERRUPT | CHANNEL_7 | LAST_ITEM_GUARD };

const uint16_t P014[] = { PIN_ANALOG | CHANNEL_9 | LAST_ITEM_GUARD };

const uint16_t P013[] = { PIN_ANALOG | CHANNEL_8 | LAST_ITEM_GUARD };

const uint16_t P012[] = { PIN_ANALOG | CHANNEL_7 | LAST_ITEM_GUARD };

const uint16_t P011[] = { PIN_ANALOG | CHANNEL_6, PIN_INTERRUPT | CHANNEL_15 | LAST_ITEM_GUARD };

const uint16_t P010[] = { PIN_ANALOG | CHANNEL_5 | LAST_ITEM_GUARD };

const uint16_t P008[] = { PIN_ANALOG | CHANNEL_14 | LAST_ITEM_GUARD };

const uint16_t P007[] = { PIN_ANALOG | CHANNEL_13 | LAST_ITEM_GUARD };

const uint16_t P006[] = { PIN_ANALOG | CHANNEL_12 | LAST_ITEM_GUARD };

const uint16_t P005[] = { PIN_ANALOG | CHANNEL_11, PIN_INTERRUPT | CHANNEL_10 | LAST_ITEM_GUARD };

const uint16_t P004[] = { PIN_ANALOG | CHANNEL_4, PIN_INTERRUPT | CHANNEL_3 | LAST_ITEM_GUARD };

const uint16_t P003[] = { PIN_ANALOG | CHANNEL_3 | LAST_ITEM_GUARD };

const uint16_t P002[] = { PIN_ANALOG | CHANNEL_2, PIN_INTERRUPT | CHANNEL_2 | LAST_ITEM_GUARD };

const uint16_t P001[] = { PIN_ANALOG | CHANNEL_1, PIN_INTERRUPT | CHANNEL_7 | LAST_ITEM_GUARD };

const uint16_t P000[] = { PIN_ANALOG | CHANNEL_0, PIN_INTERRUPT | CHANNEL_6 | LAST_ITEM_GUARD };

const uint16_t P305[] = { PIN_INTERRUPT | CHANNEL_8 | LAST_ITEM_GUARD };

const uint16_t P214[] = { LAST_ITEM_GUARD };

const uint16_t P215[] = { LAST_ITEM_GUARD };

const uint16_t P406[] = { PIN_PWM | CHANNEL_1 | PWM_CHANNEL_B | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P405[] = { PIN_PWM | CHANNEL_1 | PWM_CHANNEL_A | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P404[] = { PIN_PWM | CHANNEL_3 | PWM_CHANNEL_B | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P403[] = { PIN_PWM | CHANNEL_3 | PWM_CHANNEL_A | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P307[] = { LAST_ITEM_GUARD };

const uint16_t P306[] = { LAST_ITEM_GUARD };

const uint16_t P708[] = { LAST_ITEM_GUARD };

const uint16_t P415[] = { PIN_PWM | CHANNEL_0 | PWM_CHANNEL_A | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P414[] = { PIN_PWM | CHANNEL_0 | PWM_CHANNEL_B | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P413[] = { LAST_ITEM_GUARD };

const uint16_t P412[] = { LAST_ITEM_GUARD };

const uint16_t P808[] = { LAST_ITEM_GUARD };

const uint16_t P809[] = { LAST_ITEM_GUARD };

const uint16_t P114[] = { LAST_ITEM_GUARD };

const uint16_t P115[] = { PIN_PWM | CHANNEL_4 | PWM_CHANNEL_A | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P608[] = { PIN_PWM | CHANNEL_4 | PWM_CHANNEL_B | GPT_ODD_CFG | LAST_ITEM_GUARD };

const uint16_t P505[] = { PIN_ANALOG | CHANNEL_25, LAST_ITEM_GUARD };

const uint16_t P504[] = { PIN_ANALOG | CHANNEL_24, LAST_ITEM_GUARD };

const uint16_t P503[] = { PIN_ANALOG | CHANNEL_23, LAST_ITEM_GUARD };

const uint16_t P600[] = { LAST_ITEM_GUARD };

const uint16_t P601[] = { LAST_ITEM_GUARD };

const uint16_t P602[] = { LAST_ITEM_GUARD };

const uint16_t P603[] = { LAST_ITEM_GUARD };

const uint16_t P609[] = { LAST_ITEM_GUARD };

const uint16_t P610[] = { LAST_ITEM_GUARD };

const uint16_t P506[] = { LAST_ITEM_GUARD };

const uint16_t P203[] = { PIN_PWM | CHANNEL_5 | PWM_CHANNEL_A | GPT_ODD_CFG, SCI_CHANNEL | PIN_SCK | CHANNEL_1 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_TX_MOSI_SDA | CHANNEL_9 | SCI_ODD_CFG, PIN_MOSI | CHANNEL_1 | LAST_ITEM_GUARD };

const uint16_t P202[] = { PIN_PWM | CHANNEL_5 | PWM_CHANNEL_B | GPT_ODD_CFG, SCI_CHANNEL | PIN_CTS_RTS_SS | CHANNEL_2 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_RX_MISO_SCL | CHANNEL_9 | SCI_ODD_CFG, PIN_MISO | CHANNEL_1 | LAST_ITEM_GUARD };

const uint16_t P204[] = { PIN_PWM | CHANNEL_4 | PWM_CHANNEL_B | GPT_ODD_CFG, PIN_SCL | CHANNEL_0, SCI_CHANNEL | PIN_SCK | CHANNEL_0 | SCI_EVEN_CFG,
                          SCI_CHANNEL | PIN_SCK | CHANNEL_9 | SCI_ODD_CFG, PIN_SCK | CHANNEL_1 | LAST_ITEM_GUARD };
