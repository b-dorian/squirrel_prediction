(define (problem task)
(:domain driverlog-simple)
(:objects
    home amazon london - place
    driver - driver
    truck - truck
    mydvd - item
    myhouse - place
    dorian - dorian
)
(:init

    (at dorian home)
    (at driver home)
    (at truck amazon)
    (at mydvd amazon)

    (link amazon london)
    (link london amazon)
    (link london myhouse)
    (link myhouse london)

    (path home amazon)
    (path amazon home)

)
(:goal (and
    (at mydvd myhouse)
))
)
