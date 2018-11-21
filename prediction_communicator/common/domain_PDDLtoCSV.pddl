(define (domain driverlog-simple)
(:requirements :typing) 
(:types
    place locatable - object
    driver truck item - locatable
    biker - driver
    dorian - biker
)
(:predicates 
   
    (at ?i - locatable ?l - place)
    (in ?i - locatable ?t - truck)

    (link ?x ?y - place)
    (path ?x ?y - place)		
)


(:action LOAD-TRUCK
  :parameters
    (?i - item ?truck - truck ?l - place)
  :precondition
    (and (at ?truck ?l)
         (at ?i ?l)
    )
  :effect
   (and (not (at ?i ?l))
        (in ?i ?truck)
   )
)

(:action UNLOAD-TRUCK
  :parameters
   (?i - item ?truck - truck ?l - place)
  :precondition
   (and (at ?truck ?l)
        (in ?i ?truck)
   )
  :effect
   (and (not (in ?i ?truck))
        (at ?i ?l)
   )
)

(:action BOARD-TRUCK
  :parameters
   (?driver - driver ?truck - truck ?l - place)
  :precondition
   (and (at ?truck ?l)
        (at ?driver ?l)
   )
  :effect
   (and (not (at ?driver ?l))
        (in ?driver ?truck)
   )
)

(:action GET-OUT
  :parameters
   (?driver - driver ?truck - truck ?l - place)
  :precondition
   (and (at ?truck ?l)
        (in ?driver ?truck)
   )
  :effect
   (and (not (in ?driver ?truck))
        (at ?driver ?l)
   )
)

(:action DRIVE-TRUCK
  :parameters
   (?truck - truck ?l-from - place ?l-to - place ?driver - driver)
  :precondition
   (and (at ?truck ?l-from)
        (in ?driver ?truck)
        (link ?l-from ?l-to)
   )
  :effect
   (and (not (at ?truck ?l-from))
        (at ?truck ?l-to)
   )
)

(:action WALK
  :parameters
   (?driver - driver  ?l-from - place  ?l-to - place)
  :precondition
   (and (at ?driver ?l-from)
        (path ?l-from ?l-to)
   )
  :effect
   (and (not (at ?driver ?l-from))
        (at ?driver ?l-to)
   )
)

 
)
