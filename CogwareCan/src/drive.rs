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
/// A few minutes, because a car takes a few minutes to do this. The first
/// version ran the whole thing in forty seconds and read as a needle being
/// thrown about rather than a car being driven -- and the shut-throttle tail
/// left everything standing still long enough to look frozen.
pub const LAP_S: f32 = CYCLE_S;

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

/// How hard it pulls and how hard it stops, in km/h per second.
///
/// About 2.6 m/s² and 3.3 m/s²: brisk without being a launch, and firm
/// braking without being an emergency stop. This is the pair that decides
/// whether the speedometer looks like a car or like a slider being dragged.
const ACCEL_KMH_S: f32 = 9.5;
const BRAKE_KMH_S: f32 = 12.0;

/// The drive: how long to spend heading for each speed, in seconds and km/h.
///
/// A town run, a junction, a slip road, a stretch of dual carriageway, then
/// back off and stop. Nothing here sits still for long -- a dash with every
/// needle parked tells you nothing about whether it is still live.
const LEGS: &[(f32, f32)] = &[
    (10.0, 0.0),   // stood at idle, warming
    (25.0, 48.0),  // away, up through the low gears
    (30.0, 48.0),  // through the town
    (12.0, 20.0),  // slowing for a junction
    (10.0, 20.0),  // crawling round it
    (30.0, 100.0), // onto the slip road and up to speed
    (45.0, 105.0), // cruising
    (18.0, 62.0),  // easing off
    (25.0, 88.0),  // and back up to overtake
    (20.0, 0.0),   // braking to a stop
    (15.0, 0.0),   // stood again
];

/// The whole cycle, in seconds.
const CYCLE_S: f32 = {
    let mut total = 0.0;
    let mut i = 0;
    while i < LEGS.len() {
        total += LEGS[i].0;
        i += 1;
    }
    total
};

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

    /// The speed the drive is asking for at this point in the cycle.
    fn target_kmh(&self) -> f32 {
        let mut at = self.at_s;
        for (secs, kmh) in LEGS {
            if at < *secs {
                return *kmh;
            }
            at -= secs;
        }
        // Past the end only by rounding; the last leg is a standstill anyway.
        LEGS.last().map_or(0.0, |(_, kmh)| *kmh)
    }

    /// The leg's target, with the wander any real right foot has in it.
    ///
    /// Nobody holds a speed to the decimal. Without this the cruising legs sat
    /// at exactly one number and every needle stopped dead, which reads as a
    /// feed that has died rather than a car holding station.
    fn wandering_target(&self) -> f32 {
        let base = self.target_kmh();
        if base > 5.0 {
            base + sinf(self.at_s * 0.55) * 1.7 + sinf(self.at_s * 0.19) * 1.1
        } else {
            base
        }
    }

    /// One step of the drive.
    ///
    /// Speed chases the leg's target under real acceleration limits, and
    /// everything else follows from speed and from how hard it is working --
    /// which is what makes the numbers agree with each other rather than each
    /// sweeping on a clock of its own.
    fn drive(&mut self, dt: f32) {
        let target = self.wandering_target();

        // Chase it, but only as fast as a car can. The limit is the whole
        // reason this reads as driving rather than as a value being set.
        let gap = target - self.kmh;
        let step = if gap > 0.0 {
            (ACCEL_KMH_S * dt).min(gap)
        } else {
            -(BRAKE_KMH_S * dt).min(-gap)
        };
        self.kmh = (self.kmh + step).max(0.0);

        // Throttle is what it takes: enough to hold this speed against drag,
        // plus whatever is being asked for on top. Shut when slowing.
        let holding = 6.0 + self.kmh * 0.26;
        self.throttle = if gap > 0.5 {
            (holding + 34.0).min(92.0)
        } else if gap < -0.5 {
            0.0
        } else {
            holding
        }
        .clamp(0.0, 100.0);

        // The gear is the lowest one that still reaches this speed by its
        // change-up point, so revs fall as each goes in and climb again after.
        self.gear = if self.kmh < 3.0 {
            0
        } else {
            GEAR_TOP_KMH
                .iter()
                .position(|top| self.kmh <= *top)
                .map_or(GEAR_TOP_KMH.len(), |i| i + 1) as u8
        };
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
];

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
