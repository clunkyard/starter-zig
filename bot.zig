// A robot on the shared core Zig binding.
// It searches with a broad scan, narrows the beam on a contact, and leads
// shots from a line through its recent sightings.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
//   from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
const std=@import("std");
const cy=@import("clunkyard");
// Bearing is earned: a scan contact's bearing is off by up to its bearing_error
// (degrees, a fixed share of the arc), so a wide beam finds a robot and a narrow
// one aims at it. Contacts within 120 units are exact.
const SEARCH_ARC=40.0; // degrees: sweeps the circle in half a second
const HOLD_WIDTH=60.0; // units across at the target: it stays in the beam from tick to tick
const MIN_ARC=6.0;     // degrees: the narrowest the beam holds
const WINDOW=12;       // ticks of sightings the aim averages
const SIGHTINGS=8;     // narrow sightings in the window before a shot
const LOOK_EVERY=40;   // ticks between looks around the target, for a nearer robot
const REACQUIRE=10;    // ticks to look again where a lost target was
// A straight line through the narrowest of the last WINDOW ticks' sightings:
// where the robot is now and how fast it moves.
const Fit=struct {position:cy.Vec3,velocity:cy.Vec3,count:usize};
const Starter=struct {
    free:bool=false,dodge_ticks:u32=0,dodges:u32=0,slot:i32=0,
    // Each sighting is off by its bearing_error; the line averages that out.
    fn fit(robot:*cy.Robot,slot:i32,tick:i32) !?Fit {
        const all=try robot.tracker.history(slot);defer robot.core.allocator.free(all);
        var recent:[32]cy.Sighting=undefined;var seen:[32]cy.Sighting=undefined;var count:usize=0;var n:usize=0;var best=std.math.inf(f64);
        for(all) |s| {if(s.tick>tick-WINDOW){recent[count]=s;count+=1;best=@min(best,s.bearing_error);}}
        if(count<2)return null;
        for(recent[0..count]) |s| {if(s.bearing_error<=1.5*best){seen[n]=s;n+=1;}}
        if(n<2)return null;
        const total:f64=@floatFromInt(n);var t0:f64=0;var spread:f64=0;
        for(seen[0..n]) |s| t0+=@as(f64,@floatFromInt(s.tick))/total;
        for(seen[0..n]) |s| spread+=(@as(f64,@floatFromInt(s.tick))-t0)*(@as(f64,@floatFromInt(s.tick))-t0);
        const rate:f64=@floatFromInt(try robot.match.tick_rate());
        var now:[3]f64=undefined;var slope:[3]f64=undefined;
        for(0..3) |axis| {
            var mean:f64=0;var along:f64=0;
            for(seen[0..n]) |s| mean+=(if(axis==0) s.position.x else if(axis==1) s.position.y else s.position.z)/total;
            for(seen[0..n]) |s| along+=(@as(f64,@floatFromInt(s.tick))-t0)*((if(axis==0) s.position.x else if(axis==1) s.position.y else s.position.z)-mean);
            slope[axis]=along/spread;now[axis]=mean+slope[axis]*(@as(f64,@floatFromInt(tick))-t0);
        }
        return Fit{.position=.{.x=now[0],.y=now[1],.z=now[2]},.velocity=.{.x=slope[0]*rate,.y=slope[1]*rate,.z=slope[2]*rate},.count=n};
    }
    fn steer(robot:*cy.Robot,towards:f64,current:f64) !f64 {return try robot.angles.clamp((try robot.angles.angle_diff(towards,current))/45,-1,1);}
    pub fn onStart(bot:*@This(),robot:*cy.Robot) !void {bot.free=(try robot.match.is_3d()) and (try robot.arena.gravity()).z==0;}
    pub fn onTurn(bot:*@This(),robot:*cy.Robot) !void {
        const position=try robot.me.position();const heading=try robot.me.heading();const pitch=try robot.me.pitch();
        const tick=try robot.clock.tick();const scan_arc=(try robot.scanner.last()).arc;
        try robot.drive.forces(.{.brake=0.2});
        const last_seen=if(bot.slot!=0) try robot.tracker.last_seen(bot.slot) else null;
        if(try robot.sensors.nearest()) |target| {
            bot.slot=target.slot;
            try robot.drive.set(.{.throttle=if(target.range>350) 0.7 else if(target.range<200) -0.4 else 0.2,.turn=try steer(robot,target.bearing,heading),.pitch_turn=try steer(robot,target.elevation,pitch)});
            // Halve the beam each tick until it is HOLD_WIDTH across at the target:
            // narrow enough for a precise bearing, wide enough to keep it in.
            const hold=@max(MIN_ARC,2*std.math.atan2(HOLD_WIDTH/2,target.range)*180/std.math.pi);
            // In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks, widen it
            // for a tick: a nearer robot it finds becomes the target.
            const arc=if(@mod(tick,LOOK_EVERY)==0) SEARCH_ARC else @max(hold,@min(SEARCH_ARC,scan_arc/2));
            try robot.scanner.set(.{.heading=target.bearing,.arc=arc,.pitch=target.elevation});
            const power=try robot.gun.power_for_flight_time(target.range,1);
            // An exact contact: gun.lead is the intercept if the target holds course.
            // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
            var ready=true;
            const aim=if(target.bearing_error>0) blk: {
                const line=try fit(robot,target.slot,tick);
                const solved=if(line) |l| try robot.gun.lead_point(l.position,l.velocity,power) else null;
                ready=solved!=null and line.?.count>=SIGHTINGS;
                break :blk solved;
            } else try robot.gun.lead(target,power);
            const towards=if(aim) |a| a.heading else target.bearing;const climb=if(aim) |a| a.pitch else target.elevation;
            try robot.gun.aim(.{.heading=towards,.pitch=climb});const now=try robot.gun.aim_now();
            if(ready and @abs(try robot.angles.angle_diff(towards,now.heading))<3 and @abs(climb-now.pitch)<3 and try robot.gun.can_fire(power,10))try robot.gun.fire(.{.power=power});
            try robot.status.set(.{.text=if(ready) "TRACK" else "NARROW",.value=target.range});
        }else if(last_seen!=null and tick-last_seen.?<=REACQUIRE) {
            // Lost it: look again where it was last seen, widening the beam, before searching.
            const where=(try robot.tracker.position(bot.slot)) orelse position;
            const bearing=try robot.angles.bearing_to(position,where);const elevation=if(bot.free) try robot.angles.elevation_to(position,where) else 0;
            try robot.drive.set(.{.throttle=0.5,.turn=try steer(robot,bearing,heading),.pitch_turn=try steer(robot,elevation,pitch)});
            try robot.scanner.set(.{.heading=bearing,.arc=@min(SEARCH_ARC,scan_arc*2),.pitch=elevation});try robot.status.set(.{.text="REACQUIRE"});
        }else if(try robot.tracker.cue()) |cue| {
            const bearing=try robot.angles.bearing_to(position,cue.position);const distance=try robot.angles.distance(position,cue.position);
            const elevation=if(bot.free) try robot.angles.elevation_to(position,cue.position) else 0;const err=@min(cue.bearing_error,30);
            const across=@as(f64,@floatFromInt(@mod(tick,5)-2))/2*err;const up=if(bot.free) @as(f64,@floatFromInt(@mod(@divTrunc(tick,5),3)-1))*err/2 else 0;
            try robot.drive.set(.{.throttle=0.7,.turn=try steer(robot,bearing,heading),.pitch_turn=try steer(robot,elevation,pitch)});
            try robot.scanner.set(.{.heading=bearing+across,.arc=try robot.scanner.arc_for_reach(distance+100),.pitch=elevation+up});try robot.status.set(.{.text="CUE",.value=distance});
        }else{
            try robot.drive.set(.{.throttle=0.5,.turn=0.33,.pitch_turn=try steer(robot,0,pitch)});
            const height=if(bot.free) @as(f64,@floatFromInt(@mod(@divTrunc(tick,10),3)-1))*35 else 0;
            try robot.scanner.set(.{.heading=try robot.angles.normalize((try robot.scanner.last()).heading+36),.arc=SEARCH_ARC,.pitch=height});try robot.status.set(.{.text="SEARCH"});
        }
        // A simple dodge: vary it, or predict the shooter.
        const cues=try robot.sensors.cues();defer robot.core.allocator.free(cues);
        var fired=false;for(cues) |heard| fired=fired or heard.source==cy.Source.muzzle;
        if(bot.dodge_ticks==0 and fired){bot.dodges+=1;bot.dodge_ticks=10;}
        if(bot.dodge_ticks>0){
            // Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
            const left=bot.dodges%2==1;const side:f64=if(left) 1 else -1;bot.dodge_ticks-=1;
            if(try robot.match.control()==cy.Control.forces){try robot.drive.set(.{.throttle=0.5});try robot.drive.forces(.{.strafe=-side});}
            else try robot.drive.set(.{.throttle=1,.turn=side});
            try robot.status.set(.{.text=if(left) "dodge:left" else "dodge:right",.value=@floatFromInt(bot.dodges)});
        }
        if(try robot.battery.charge()<(try robot.battery.capacity())/4)try robot.drive.set(.{.throttle=((try robot.drive.get_throttle()) orelse 0)*0.5});
        if(try robot.nav.storm()) |storm| {if(((try storm.margin(position)) orelse -1e9) > -60)try robot.drive.set(.{.throttle=0.8,.turn=try robot.angles.clamp((try robot.angles.angle_diff(try robot.angles.bearing_to(storm.center,position),heading))/30,-1,1)});}
        if(try robot.nav.zone()) |zone| {if(((try zone.margin(position)) orelse 1e9)<60){const climb=if(zone.shape==cy.Shape.sphere) try robot.angles.elevation_to(position,zone.center) else 0;try robot.drive.set(.{.throttle=0.8,.turn=try robot.angles.clamp((try robot.angles.angle_diff(try robot.angles.bearing_to(position,zone.center),heading))/30,-1,1),.pitch_turn=try robot.angles.clamp((climb-pitch)/30,-1,1)});}}
    }
};
pub fn main() !void {try cy.run(Starter);}
