function machNumber {
    parameter idx is 1.4.
    if not body:atm:exists or body:atm:altitudepressure(altitude) = 0 {
        return 0.
    }
    return round(sqrt(2 / idx * ship:q / body:atm:altitudepressure(altitude)),3).
}
