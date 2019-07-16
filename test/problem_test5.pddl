(define (problem test1)
(:domain default_domain)
(:objects
  maindoor_to_corridor corridor_to_maindoor corridor_to_livingroom livingroom_to_corridor - door
  outdoor corridor kitchen livingroom bedroom bathroom - room
  wp_maindoor_to_corridor wp_corridor_from_maindoor wp_corridor_to_maindoor wp_maindoor_from_corridor wp_corridor_to_livingroom wp_livingroom_from_corridor wp_livingroom_to_corridor wp_corridor_from_livingroom wp_corridor wp_livingroom wp_kitchen wp_bedroom wp_bathroom - waypoint
  juan - person
  leia - robot
)
(:init
 (door_connected maindoor_to_corridor outdoor corridor wp_maindoor_to_corridor wp_corridor_from_maindoor)
 (door_connected corridor_to_maindoor corridor outdoor wp_corridor_to_maindoor wp_maindoor_from_corridor)
 (door_connected corridor_to_livingroom corridor livingroom wp_corridor_to_livingroom wp_livingroom_from_corridor)
 (door_connected livingroom_to_corridor livingroom corridor wp_livingroom_to_corridor wp_corridor_from_livingroom)
 (door_opened maindoor_to_corridor)
 (door_opened corridor_to_maindoor)
 (door_opened corridor_to_livingroom)
 (door_opened livingroom_to_corridor)
 (free_connected corridor kitchen)
 (free_connected kitchen corridor)
 (free_connected kitchen bedroom)
 (free_connected bedroom kitchen)
 (free_connected livingroom bedroom)
 (free_connected bedroom livingroom)
 (free_connected livingroom bathroom)
 (free_connected bathroom livingroom)
 (robot_at leia wp_maindoor_to_corridor)
 (robot_at_room leia outdoor)
 (person_at juan wp_kitchen)
 (person_at_room juan kitchen)
 (waypoint_at wp_maindoor_to_corridor outdoor)
 (waypoint_at wp_corridor_from_maindoor corridor)
 (waypoint_at wp_corridor_to_maindoor corridor)
 (waypoint_at wp_maindoor_from_corridor outdoor)
 (waypoint_at wp_corridor_to_livingroom corridor)
 (waypoint_at wp_livingroom_from_corridor livingroom)
 (waypoint_at wp_livingroom_to_corridor livingroom)
 (waypoint_at wp_corridor_from_livingroom corridor)
 (waypoint_at wp_corridor corridor)
 (waypoint_at wp_livingroom livingroom)
 (waypoint_at wp_kitchen kitchen)
 (waypoint_at wp_bedroom bedroom)
 (waypoint_at wp_bathroom bathroom)
 (person_at juan wp_kitchen)
)
(:goal (and
    (person_at juan wp_maindoor_to_corridor)
)))
