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
/// Long enough that a gear holds for a moment and somebody watching can see
/// what is happening, short enough to sit through while adjusting a widget.
pub const LAP_S: f32 = 40.0;

/// Redline, and the shift point just under it.
const REDLINE: f32 = 7000.0;
const SHIFT_AT: f32 = 6200.0;
/// Where the engine settles with no throttle.
const IDLE_RPM: f32 = 850.0;

/// Road speed in each gear at [`SHIFT_AT`], which is what sets the ratios.
///
/// Five numbers rather than a gearbox: what matters on the glass is that
/// speed keeps climbing while revs drop back on every shift.
const GEAR_TOP_KMH: [f32; 5] = [40.0, 70.0, 105.0, 145.0, 190.0];

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
        Some((physical * spec.unit.scale as f32).round() as i32)
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

    /// One step of the lap.
    ///
    /// Throttle comes from where the lap is; everything else follows from
    /// throttle and speed, which is what makes the numbers agree with each
    /// other rather than each sweeping on its own clock.
    fn drive(&mut self, dt: f32) {
        let phase = self.at_s / LAP_S;
        // Pull away, hold it open through the gears, lift for the cruise,
        // then shut and brake back to a standstill.
        self.throttle = match phase {
            p if p < 0.06 => 0.0,           // sat at idle
            p if p < 0.45 => 85.0,          // away and up through the gears
            p if p < 0.70 => 22.0,          // cruising
            p if p < 0.90 => 0.0,           // shut off, slowing
            _ => 0.0,                       // stopped again
        };

        // Speed follows throttle, with drag and braking taking it back.
        let pull = self.throttle * 0.11;
        let drag = self.kmh * 0.012 + if self.throttle == 0.0 { 6.0 } else { 0.0 };
        self.kmh = (self.kmh + (pull - drag) * dt * 6.0).clamp(0.0, GEAR_TOP_KMH[4]);

        // The gear is whichever one holds this speed under the shift point,
        // so revs fall as each goes in and climb again after.
        self.gear = if self.kmh < 2.0 {
            0
        } else {
            GEAR_TOP_KMH
                .iter()
                .position(|top| self.kmh <= *top)
                .map_or(GEAR_TOP_KMH.len(), |i| i + 1) as u8
        };
        self.rpm = match self.gear {
            0 => IDLE_RPM + self.throttle * 12.0,
            g => {
                let top = GEAR_TOP_KMH[usize::from(g) - 1];
                let bottom = if g > 1 { GEAR_TOP_KMH[usize::from(g) - 2] } else { 0.0 };
                let through = ((self.kmh - bottom) / (top - bottom)).clamp(0.0, 1.0);
                (IDLE_RPM + through * (SHIFT_AT - IDLE_RPM)).min(REDLINE)
            }
        };

        // Manifold pressure tracks the throttle plate: near ambient open,
        // pulled down towards vacuum closed.
        self.map = 100.0 - (100.0 - self.throttle) * 0.72;

        // Coolant warms to its thermostat and stays; oil follows it a little
        // behind, which is the pair somebody watches on a cold start.
        self.coolant += (THERMOSTAT_C - self.coolant) * dt * 0.05;
        self.oil_temp += (self.coolant - 6.0 - self.oil_temp) * dt * 0.03;
        // Intake sits above ambient by however much the engine is heating it.
        self.intake = AMBIENT_C + 14.0 - (self.kmh * 0.06).min(11.0);

        // Oil pressure is mostly a function of revs, with a floor at idle.
        self.oil_pres = 110.0 + (self.rpm / REDLINE) * 310.0;

        // The alternator is charging once it is off idle; a little sag under
        // load keeps the voltmeter from looking painted on.
        self.volts = 13.9 + (self.rpm / REDLINE) * 0.5 - self.throttle * 0.004;

        // A tank does not move much in forty seconds, so this is slow enough
        // to be honest and fast enough to prove the gauge is live.
        self.fuel = (self.fuel - dt * 0.05).max(4.0);
    }
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
}
