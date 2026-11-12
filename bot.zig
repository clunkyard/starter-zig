// A robot on the shared core Zig binding.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
const cy=@import("clunkyard");
const Starter=struct {
    free:bool=false,
    fn steer(robot:*cy.Robot,towards:f64,current:f64) !f64 {return try robot.angles.clamp((try robot.angles.angle_diff(towards,current))/45,-1,1);}
    pub fn onStart(bot:*@This(),robot:*cy.Robot) !void {bot.free=(try robot.match.is_3d()) and (try robot.arena.gravity()).z==0;}
    pub fn onTurn(bot:*@This(),robot:*cy.Robot) !void {
        const position=try robot.me.position();const heading=try robot.me.heading();const pitch=try robot.me.pitch();
        try robot.drive.forces(.{.brake=0.2});
        if(try robot.sensors.nearest()) |target| {
            try robot.drive.set(.{.throttle=if(target.range>350) 0.7 else if(target.range<200) -0.4 else 0.2,.turn=try steer(robot,target.bearing,heading),.pitch_turn=try steer(robot,target.elevation,pitch)});
            try robot.scanner.set(.{.heading=target.bearing,.arc=20,.pitch=target.elevation});
            const power=try robot.gun.power_for_flight_time(target.range,1);const aim=try robot.gun.lead(target,power);
            const towards=if(aim) |a| a.heading else target.bearing;const climb=if(aim) |a| a.pitch else target.elevation;
            try robot.gun.aim(.{.heading=towards,.pitch=climb});const now=try robot.gun.aim_now();
            if(@abs(try robot.angles.angle_diff(towards,now.heading))<3 and @abs(climb-now.pitch)<3 and try robot.gun.can_fire(power,10))try robot.gun.fire(.{.power=power});
            try robot.status.set(.{.text="TRACK",.value=target.range});
        }else if(try robot.tracker.cue()) |cue| {
            const bearing=try robot.angles.bearing_to(position,cue.position);const distance=try robot.angles.distance(position,cue.position);
            const elevation=if(bot.free) try robot.angles.elevation_to(position,cue.position) else 0;const err=@min(cue.bearing_error,30);const tick=try robot.clock.tick();
            const across=@as(f64,@floatFromInt(@mod(tick,5)-2))/2*err;const up=if(bot.free) @as(f64,@floatFromInt(@mod(@divTrunc(tick,5),3)-1))*err/2 else 0;
            try robot.drive.set(.{.throttle=0.7,.turn=try steer(robot,bearing,heading),.pitch_turn=try steer(robot,elevation,pitch)});
            try robot.scanner.set(.{.heading=bearing+across,.arc=try robot.scanner.arc_for_reach(distance+100),.pitch=elevation+up});try robot.status.set(.{.text="CUE",.value=distance});
        }else{
            try robot.drive.set(.{.throttle=0.5,.turn=0.33,.pitch_turn=try steer(robot,0,pitch)});
            const height=if(bot.free) @as(f64,@floatFromInt(@mod(@divTrunc(try robot.clock.tick(),10),3)-1))*35 else 0;
            try robot.scanner.set(.{.heading=try robot.angles.normalize((try robot.scanner.last()).heading+36),.arc=40,.pitch=height});try robot.status.set(.{.text="SEARCH"});
        }
        if(try robot.battery.charge()<(try robot.battery.capacity())/4)try robot.drive.set(.{.throttle=((try robot.drive.get_throttle()) orelse 0)*0.5});
        if(try robot.nav.storm()) |storm| {if(((try storm.margin(position)) orelse -1e9) > -60)try robot.drive.set(.{.throttle=0.8,.turn=try robot.angles.clamp((try robot.angles.angle_diff(try robot.angles.bearing_to(storm.center,position),heading))/30,-1,1)});}
        if(try robot.nav.zone()) |zone| {if(((try zone.margin(position)) orelse 1e9)<60){const climb=if(zone.shape==cy.Shape.sphere) try robot.angles.elevation_to(position,zone.center) else 0;try robot.drive.set(.{.throttle=0.8,.turn=try robot.angles.clamp((try robot.angles.angle_diff(try robot.angles.bearing_to(position,zone.center),heading))/30,-1,1),.pitch_turn=try robot.angles.clamp((climb-pitch)/30,-1,1)});}}
    }
};
pub fn main() !void {try cy.run(Starter);}
