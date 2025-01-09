解析函数的结构是这样的

| Prev frame |     0XAE     |  0xEA | length |  cmd  |  data |       |       |                           |  data |       |       |       |       |       |       |       |       |       |       | checksum |  0XEF |  0xFE | Next frame |
|:----------:|:------------:|:-----:|:------:|:-----:|:-----:|:-----:|:-----:|:-------------------------:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|:--------:|:-----:|:-----:|:----------:|
|            | ^ buffer_loc |       |        |       |       |       |       | ^ i = 4 + buffer_loc; i++ |       |       |       |       |       |       |       |       |       |       |       |          |       |       |            |
|    1Byte   |     1Byte    | 1Byte |  1Byte | 1Byte | 1Byte | 1Byte | 1Byte |           1Byte           | 1Byte | 1Byte | 1Byte | 1Byte | 1Byte | 1Byte | 1Byte | 1Byte | 1Byte | 1Byte | 1Byte |   1Byte  | 1Byte | 1Byte |    1Byte   |

