def can_hex_to_pgn_sa(id_hex):
    id = int(id_hex, 16)
    return can_id_to_pgn_sa(id)    

def can_id_to_pgn_sa(id):
    try:
        extended = id >> 29 > 0
        priority = id >> 26 & int('111', 2)
        reserved = id >> 25 & 1
        data_page = id >> 24 & 1
        pdu_format = id >> 16 & int('11111111', 2)
        pdu_specific = id >> 8 & int('11111111', 2)
        source_address = id >> 0 & int('11111111', 2)
        specific_shift = pdu_specific if pdu_format >= 240 else 0
        pgn = reserved << 17 | data_page << 16 | pdu_format << 8 | specific_shift
        return (pgn, source_address)
    except:
        return (-1, -1)

def pgn_sa_to_can_hex(pgn, sa, priority=6, extended=True):
    id = pgn_sa_to_can_id(pgn, sa, priority, extended)
    return hex(id)

def pgn_sa_to_can_id(pgn, sa, priority=6, extended=True):
    ext = 4 << 29 if extended else 0
    priority = priority << 26
    reserved = 0
    data_page = pgn >> 16 & 1
    pdu_format = pgn >> 8 & int('11111111', 2)
    pdu_specific = pgn & int('11111111', 2)

    return ext | priority | reserved | data_page << 24 | pdu_format << 16 | pdu_specific << 8 | sa 


