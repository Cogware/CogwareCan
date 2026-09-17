// SPDX-License-Identifier: GPL-3.0-only
//! A car that is not there, driven in a loop.
//!
//! A sweep from empty to full proves a needle reaches both ends and nothing
//! else. It never shows a tachometer dropping as a gear goes in, coolant
//! sitting at ninety while everything else moves, or oil pressure following
//! revs -- which is what a dash actually looks like, and what somebody laying
//! one out needs to see.
//!
//! So this drives a lap instead: pull away, work up the gears, cruise, brake
//! back to idle, and round again. Nothing here is a real engine model. It is
//! the smallest set of relationships that make the numbers move together
//! rather than independently, which is the part a sweep gets wrong.
//!
//! ```ignore
//! let mut car = Car::new();
//! car.advance(delta_us);
//! car.write();                       // into the gauge table
//! let rpm = car.reading(RPM.id);     // or one at a time
//! ```

use crate::gauge::{gauge_by_id, GaugeSpec};

/// How long one lap takes, in seconds.
///
/// However long the trace it follows runs for. Ten minutes of real driving,
/// which is what it takes to look like driving: the first two attempts at
/// writing a cycle by hand were either thrown about or standing still, and
/// neither read as a car.
pub const LAP_S: f32 = crate::trace::SECONDS as f32;

/// Redline, and where it changes up in normal driving.
///
/// A road car shifts a long way short of the limiter unless it is being
/// pressed, so the needle lives in the middle of the dial rather than the top.
const REDLINE: f32 = 7000.0;
const SHIFT_AT: f32 = 3200.0;
/// Where the engine settles with no throttle.
const IDLE_RPM: f32 = 820.0;

/// Road speed at which each gear reaches [`SHIFT_AT`].
///
/// Five numbers rather than a gearbox: what matters on the glass is that road
/// speed keeps climbing while the revs drop back on every change.
const GEAR_TOP_KMH: [f32; 5] = [22.0, 40.0, 62.0, 92.0, 135.0];

/// A whole dashboard's worth of readings, as physical quantities.
///
/// Physical, not the fixed-point counts the bus carries: the scaling belongs
/// to the gauge table, and doing it here would put the same constants in two
/// places.
#[derive(Clone, Copy, Debug, Default)]
pub struct Car {
    /// Seconds into the lap.
    at_s: f32,
    /// Engine speed, rpm.
    pub rpm: f32,
    /// Road speed, km/h.
    pub kmh: f32,
    /// Which gear, 1..=5, or 0 for neutral at rest.
    pub gear: u8,
    /// Throttle, 0..=100.
    pub throttle: f32,
    /// Coolant, degrees C.
    pub coolant: f32,
    /// Oil, degrees C, which lags coolant.
    pub oil_temp: f32,
    /// Oil pressure, kPa.
    pub oil_pres: f32,
    /// Manifold pressure, kPa.
    pub map: f32,
    /// Intake air, degrees C.
    pub intake: f32,
    /// Battery, volts.
    pub volts: f32,
    /// What is left in the tank, 0..=100.
    pub fuel: f32,
    /// What the engine is actually running, as an air-fuel ratio.
    pub afr: f32,
    /// What the ECU is aiming for.
    pub afr_target: f32,
    /// Boost the ECU is asking for, kPa above nothing.
    pub boost_target: f32,
}

impl Car {
    /// A cold car at rest, at the start of a lap.
    #[must_use]
    pub fn new() -> Self {
        Self {
            coolant: AMBIENT_C,
            oil_temp: AMBIENT_C,
            intake: AMBIENT_C,
            volts: 12.4,
            fuel: 78.0,
            afr: STOICH,
            afr_target: STOICH,
            boost_target: 0.0,
            ..Self::default()
        }
    }

    /// Move the car on by `delta_us` microseconds.
    pub fn advance(&mut self, delta_us: u64) {
        let dt = delta_us as f32 / 1_000_000.0;
        self.at_s = (self.at_s + dt) % LAP_S;
        self.drive(dt);
    }

    /// Where the lap is, 0.0..=1.0, for anything that wants to show progress.
    #[must_use]
    pub fn through_lap(&self) -> f32 {
        self.at_s / LAP_S
    }

    /// One gauge's reading in the unit the bus carries it in, or `None` for a
    /// gauge this does not simulate.
    ///
    /// By id rather than by name so a caller holding a subscription can ask
    /// for exactly what it subscribed to.
    #[must_use]
    pub fn reading(&self, id: u16) -> Option<i32> {
        let spec = gauge_by_id(id)?;
        let physical = self.physical(spec)?;
        let scaled = physical * spec.unit.scale as f32;
        // Rounded by hand: `f32::round` is std and this crate is not. Half
        // away from zero, so a reading below freezing rounds the same way one
        // above it does rather than towards it.
        let half = if scaled < 0.0 { -0.5 } else { 0.5 };
        Some((scaled + half) as i32)
    }

    /// Write every gauge this simulates into the table.
    pub fn write(&self) {
        for (id, _) in DRIVEN {
            if let (Some(g), Some(v)) = (gauge_by_id(*id), self.reading(*id)) {
                g.set(v);
            }
        }
    }

    /// The physical value for one gauge, before the unit's scale.
    fn physical(&self, spec: &GaugeSpec) -> Option<f32> {
        DRIVEN
            .iter()
            .find(|(id, _)| *id == spec.id)
            .map(|(_, pick)| pick(self))
    }

    /// Road speed from the trace, interpolated between its one-second samples.
    ///
    /// Straight from the log rather than chased with an acceleration limit:
    /// the accelerations in it are already a real car's, and smoothing them
    /// towards a target is what made the hand-written cycles feel synthetic.
    fn traced_kmh(&self) -> f32 {
        let samples = crate::trace::SPEED_DKMH;
        let at = self.at_s.max(0.0);
        let i = at as usize;
        let (a, b) = match (samples.get(i), samples.get(i + 1)) {
            (Some(a), Some(b)) => (*a as f32, *b as f32),
            (Some(a), None) => (*a as f32, *a as f32),
            _ => return 0.0,
        };
        let frac = at - i as f32;
        (a + (b - a) * frac) / 10.0
    }

    /// Which gear to be in, holding the one already engaged until the speed
    /// is properly past a change point.
    ///
    /// Without the margin it hunts: a trace sitting either side of a boundary
    /// swaps gear every second and throws the tachometer between two very
    /// different numbers, which no gearbox does and no dash should show.
    fn choose_gear(&self) -> u8 {
        const MARGIN_KMH: f32 = 4.0;
        if self.kmh < 3.0 {
            return 0;
        }
        let ideal = GEAR_TOP_KMH
            .iter()
            .position(|top| self.kmh <= *top)
            .map_or(GEAR_TOP_KMH.len(), |i| i + 1) as u8;
        let held = self.gear;
        if held == 0 || held > GEAR_TOP_KMH.len() as u8 {
            return ideal;
        }
        let top = GEAR_TOP_KMH[usize::from(held) - 1];
        let bottom = if held > 1 { GEAR_TOP_KMH[usize::from(held) - 2] } else { 0.0 };
        // Stay put while the speed is still within this gear's band, widened
        // at both ends so crossing a boundary has to mean it.
        if self.kmh <= top + MARGIN_KMH && self.kmh >= bottom - MARGIN_KMH {
            held
        } else {
            ideal
        }
    }

    /// One step of the drive.
    ///
    /// Speed chases the leg's target under real acceleration limits, and
    /// everything else follows from speed and from how hard it is working --
    /// which is what makes the numbers agree with each other rather than each
    /// sweeping on a clock of its own.
    fn drive(&mut self, dt: f32) {
        let was = self.kmh;
        self.kmh = self.traced_kmh();
        // What the car is actually doing, in km/h per second. The trace's own
        // derivative, so hard acceleration and lifting off are the real ones.
        let accel = if dt > 0.0 { (self.kmh - was) / dt } else { 0.0 };

        // Throttle is what it takes: enough to hold this speed against drag,
        // plus whatever the acceleration is asking for on top, and shut when
        // the trace is slowing.
        let holding = 6.0 + self.kmh * 0.26;
        self.throttle = if accel < -0.4 {
            0.0
        } else {
            holding + accel * 7.0
        }
        .clamp(0.0, 100.0);

        // The gear is the lowest one that still reaches this speed by its
        // change-up point, so revs fall as each goes in and climb again after.
        self.gear = self.choose_gear();
        self.rpm = match self.gear {
            // Idle wanders a little, the way one does. A needle that is
            // perfectly still is one nothing is feeding.
            0 => IDLE_RPM + sinf(self.at_s * 1.7) * 25.0 + self.throttle * 8.0,
            g => {
                let top = GEAR_TOP_KMH[usize::from(g) - 1];
                let bottom = if g > 1 { GEAR_TOP_KMH[usize::from(g) - 2] } else { 0.0 };
                let through = ((self.kmh - bottom) / (top - bottom)).clamp(0.0, 1.0);
                let revs = IDLE_RPM + 350.0 + through * (SHIFT_AT - IDLE_RPM - 350.0);
                // Working harder holds a gear longer, so it sits higher.
                (revs + self.throttle * 4.5).min(REDLINE)
            }
        };

        // Manifold pressure tracks the throttle plate: near ambient open,
        // pulled down towards vacuum closed.
        self.map = 100.0 - (100.0 - self.throttle) * 0.72;

        // Coolant warms to its thermostat and stays; oil follows it a little
        // behind, which is the pair somebody watches on a cold start.
        self.coolant += (THERMOSTAT_C - self.coolant) * dt * 0.02;
        self.oil_temp += (self.coolant - 6.0 - self.oil_temp) * dt * 0.012;
        // Intake sits above ambient by however much the engine is heating it,
        // less whatever the road speed is carrying away.
        self.intake = AMBIENT_C + 14.0 - (self.kmh * 0.08).min(11.0);

        // Oil pressure is mostly a function of revs, with a floor at idle.
        self.oil_pres = 110.0 + (self.rpm / REDLINE) * 310.0;

        // The alternator is charging once it is off idle; a little sag under
        // load keeps the voltmeter from looking painted on.
        self.volts = 13.9 + (self.rpm / REDLINE) * 0.5 - self.throttle * 0.004;

        // What the ECU is asking for: stoichiometric while it is cruising and
        // can run closed loop, richer as soon as it is being worked, because
        // a hot engine at full throttle is cooled by fuel.
        self.afr_target = if self.throttle > 65.0 {
            11.9
        } else if self.throttle > 35.0 {
            13.2
        } else {
            STOICH
        };
        // What it is actually running chases that, and overshoots on a change
        // of throttle the way a real mixture does before the loop catches up.
        self.afr += (self.afr_target - self.afr) * dt * 4.0;
        self.afr += sinf(self.at_s * 2.3) * 0.06;
        self.afr = self.afr.clamp(10.5, 16.5);

        // Boost is asked for in proportion to how hard it is being driven,
        // and only once there are revs enough to make any.
        let spool = ((self.rpm - 2200.0) / 2200.0).clamp(0.0, 1.0);
        self.boost_target = spool * (self.throttle / 100.0) * 95.0;

        // A tank does not move much in a few minutes, so this is slow enough
        // to be honest and fast enough to prove the gauge is live.
        self.fuel = (self.fuel - dt * 0.012).max(4.0);
    }
}

/// Sine, without `std`.
///
/// Only used to make idle wander, so a short Taylor series folded into one
/// quadrant is more than close enough and saves a dependency.
fn sinf(x: f32) -> f32 {
    const TAU: f32 = core::f32::consts::TAU;
    let mut x = x % TAU;
    if x < 0.0 {
        x += TAU;
    }
    // Fold to -pi..pi, where the series is accurate.
    if x > core::f32::consts::PI {
        x -= TAU;
    }
    let x2 = x * x;
    x * (1.0 - x2 / 6.0 * (1.0 - x2 / 20.0 * (1.0 - x2 / 42.0)))
}

/// Ambient, for the temperatures that start there.
const AMBIENT_C: f32 = 18.0;
/// Where the thermostat holds coolant.
const THERMOSTAT_C: f32 = 89.0;

/// The gauges this drives, and where each reading comes from.
///
/// A table rather than a match so the same list serves writing them all and
/// answering for one, and so adding a channel is one line.
type Pick = fn(&Car) -> f32;
pub static DRIVEN: &[(u16, Pick)] = &[
    (0x24, |c| c.map),
    (0x25, |c| c.intake),
    (0x26, |c| c.coolant),
    (0x28, |c| c.volts),
    (0x2D, |c| c.rpm),
    (0x35, |c| c.throttle),
    (0x57, |c| c.kmh),
    (0x58, |c| f32::from(c.gear)),
    (0x5A, |c| c.oil_pres),
    (0x71, |c| c.oil_temp),
    (0x73, |c| c.fuel),
    (0x29, |c| c.afr),
    (0x31, |c| c.afr_target),
    (0x38, |c| c.boost_target),
];

/// A stoichiometric mixture for petrol, which is what a warm engine idles and
/// cruises at whenever the ECU is allowed to run closed loop.
const STOICH: f32 = 14.7;

#[cfg(test)]
mod tests {
    use super::*;

    /// Run a whole lap, sampling as it goes.
    fn lap() -> Vec<Car> {
        let mut car = Car::new();
        let mut seen = Vec::new();
        for _ in 0..(LAP_S as usize * 20) {
            car.advance(50_000);
            seen.push(car);
        }
        seen
    }

    #[test]
    fn it_pulls_away_and_comes_back_to_a_stop() {
        let seen = lap();
        let fastest = seen.iter().fold(0.0f32, |m, c| m.max(c.kmh));
        assert!(fastest > 80.0, "it should get up to speed, saw {fastest}");
        assert!(
            seen.last().expect("samples").kmh < 5.0,
            "and be stopped again by the end"
        );
    }

    #[test]
    fn the_revs_drop_when_a_gear_goes_in() {
        // The thing a sweep cannot show: the tachometer falling while the
        // speedometer keeps climbing.
        let seen = lap();
        let dropped = seen.windows(2).any(|w| {
            w[1].gear > w[0].gear && w[1].gear > 0 && w[1].rpm < w[0].rpm - 200.0
        });
        assert!(dropped, "a shift should drop the revs");
    }

    #[test]
    fn the_revs_stay_under_the_redline() {
        for c in lap() {
            assert!(c.rpm <= REDLINE + 1.0, "{} rpm is over the redline", c.rpm);
            assert!(c.rpm >= 0.0);
        }
    }

    #[test]
    fn it_warms_up_and_then_holds() {
        let seen = lap();
        let first = seen.first().expect("samples").coolant;
        let last = seen.last().expect("samples").coolant;
        assert!(last > first + 10.0, "coolant should climb off ambient");
        assert!(last <= THERMOSTAT_C, "and not pass the thermostat");
    }

    #[test]
    fn oil_pressure_follows_the_revs() {
        let seen = lap();
        let idling = seen.iter().min_by(|a, b| a.rpm.total_cmp(&b.rpm)).expect("samples");
        let working = seen.iter().max_by(|a, b| a.rpm.total_cmp(&b.rpm)).expect("samples");
        assert!(
            working.oil_pres > idling.oil_pres + 100.0,
            "pressure should rise with revs"
        );
    }

    #[test]
    fn every_driven_gauge_is_one_the_table_has() {
        // A channel naming an id nobody carries would be a reading that goes
        // nowhere, and the list is written by hand.
        for (id, _) in DRIVEN {
            assert!(gauge_by_id(*id).is_some(), "{id:#04x} is not in the table");
        }
    }

    #[test]
    fn a_reading_comes_back_in_the_unit_the_bus_carries() {
        let mut car = Car::new();
        for _ in 0..200 {
            car.advance(50_000);
        }
        // Coolant is tenths of a degree on the wire, so a warm engine reads
        // in the hundreds rather than the tens.
        let coolant = car.reading(0x26).expect("coolant is driven");
        assert!(
            (200..=900).contains(&coolant),
            "coolant reads {coolant} in tenths"
        );
        assert!(car.reading(0xFFFF).is_none(), "and an unknown id is none");
    }


    #[test]
    fn it_takes_a_car_s_worth_of_time_to_reach_a_speed() {
        // The first version did nought to a hundred and ninety in eighteen
        // seconds, which read as a needle being thrown rather than a car.
        let seen = lap();
        let step = LAP_S / seen.len() as f32;
        let from = seen.iter().position(|c| c.kmh >= 5.0).expect("pulls away");
        let to = seen.iter().position(|c| c.kmh >= 95.0).expect("reaches speed");
        let took = (to - from) as f32 * step;
        assert!(took > 8.0, "nought to ninety-five took {took:.1}s, too quick");
    }

    #[test]
    fn nothing_stands_still_for_long() {
        // A dash where every needle is parked tells you nothing about whether
        // it is still being fed, which is what made the old cycle look frozen.
        let seen = lap();
        let step = LAP_S / seen.len() as f32;
        let mut still = 0.0;
        let mut longest: f32 = 0.0;
        for w in seen.windows(2) {
            if (w[1].rpm - w[0].rpm).abs() < 1.0 && (w[1].kmh - w[0].kmh).abs() < 0.05 {
                still += step;
                longest = longest.max(still);
            } else {
                still = 0.0;
            }
        }
        assert!(longest < 4.0, "everything sat still for {longest:.1}s");
    }

    #[test]
    fn the_revs_sit_where_a_road_car_s_do() {
        // Not pinned to the limiter: a cycle that spends its life near the
        // redline is not what a dash looks like on the way to the shops.
        let seen = lap();
        let moving: Vec<_> = seen.iter().filter(|c| c.gear > 0).collect();
        let mean = moving.iter().map(|c| c.rpm).sum::<f32>() / moving.len() as f32;
        assert!(
            (1200.0..3200.0).contains(&mean),
            "mean of {mean:.0} rpm while moving"
        );
    }

    #[test]
    fn the_gearbox_does_not_hunt_across_a_change_point() {
        // A trace sitting either side of a boundary was swapping gear every
        // second and throwing the tachometer between two very different
        // numbers, which no gearbox does.
        let seen = lap();
        let changes = seen.windows(2).filter(|w| w[0].gear != w[1].gear).count();
        // Ten minutes of hard driving is a few dozen changes, not hundreds.
        assert!(changes < 120, "{changes} gear changes in one run");
    }

    #[test]
    fn the_trace_is_the_one_that_was_published() {
        // A table this long is generated, and a resampling mistake would be
        // invisible on the glass. Its shape is worth pinning.
        let trace = crate::trace::SPEED_DKMH;
        assert_eq!(crate::trace::SECONDS, 600, "US06 runs ten minutes");
        assert_eq!(trace[0], 0, "it starts at a standstill");
        let peak = trace.iter().copied().max().expect("samples");
        // Eighty miles an hour, in tenths of a km/h.
        assert!((1280..1300).contains(&peak), "peak of {peak} is not the US06's");
    }

    #[test]
    fn the_mixture_richens_when_it_is_worked() {
        let seen = lap();
        let hardest = seen
            .iter()
            .max_by(|a, b| a.throttle.total_cmp(&b.throttle))
            .expect("samples");
        assert!(
            hardest.afr < 13.5,
            "wide open it should be rich, saw {:.1}",
            hardest.afr
        );
        let cruising = seen
            .iter()
            .filter(|c| c.throttle > 5.0 && c.throttle < 30.0 && c.kmh > 40.0)
            .count();
        assert!(cruising > 0, "there should be some cruising in a real trace");
    }

    #[test]
    fn nothing_a_scene_binds_is_left_reading_zero() {
        // The left gauge sat at AFR 0 because the model drove eleven channels
        // and the scene bound a twelfth. A gauge nobody feeds is indis-
        // tinguishable on the glass from one reading nought.
        let mut car = Car::new();
        for _ in 0..1200 {
            car.advance(50_000);
        }
        for (id, _) in DRIVEN {
            let name = gauge_by_id(*id).expect("in the table").name;
            assert!(
                car.reading(*id).is_some(),
                "{name} is listed as driven but has no reading"
            );
        }
        // The ones the cockpit's own scenes bind, by id, so adding a widget
        // that reads something nothing produces fails here rather than on a
        // dashboard.
        for id in [0x24, 0x26, 0x2D, 0x57, 0x5A, 0x73, 0x28, 0x29, 0x31, 0x38] {
            let name = gauge_by_id(id).expect("in the table").name;
            assert!(car.reading(id).is_some(), "{name} is bound but not driven");
        }
    }

    #[test]
    fn a_reading_below_zero_rounds_away_from_zero_too() {
        // Temperatures are the signed ones, and a round that always went up
        // would bias every cold reading warm.
        let mut car = Car::new();
        car.intake = -4.26;
        assert_eq!(car.reading(0x25), Some(-43), "tenths of a degree");
        car.intake = 4.26;
        assert_eq!(car.reading(0x25), Some(43));
    }
}
