// SPDX-License-Identifier: Apache-2.0
//! What the network is built from: the tech table and one supply net, as plain data.
//!
//! The database read (`read.rs`) fills these from two flat streams the opendb shim writes in the
//! database's own iteration order; [`decode_tech`] and [`decode_net`] read them back in that order.
//! Keeping the network code on plain data lets every stage be tested without a database.

pub use vyges_drt::polygon90::Rect;

/// A routing layer's preferred direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    None,
    Horizontal,
    Vertical,
}

/// One cut-class rule of a cut layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CutClass {
    pub cuts_valid: bool,
    pub width: i32,
    pub length_valid: bool,
    pub length: i32,
    pub num_cuts: i32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Layer {
    /// The database object id: what a per-layer map is ordered by.
    pub id: u32,
    /// The layer number: what node order compares.
    pub number: i32,
    pub name: String,
    /// 0 for a cut (or masterslice) layer.
    pub routing_level: i32,
    pub pitch: i32,
    pub dir: Dir,
    pub backside: bool,
    /// The layer above, by number.
    pub upper: Option<i32>,
    pub cut_classes: Vec<CutClass>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Tech {
    pub dbus: i32,
    /// The first front-side and back-side routing layers, by number.
    pub front_base: Option<i32>,
    pub back_base: Option<i32>,
    pub layers: Vec<Layer>,
}

impl Tech {
    /// The index into [`Tech::layers`] of the layer with this number.
    pub fn by_number(&self, number: i32) -> Option<usize> {
        self.layers.iter().position(|l| l.number == number)
    }
}

/// One shape of a via, on one of its layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ViaShape {
    /// Layer number.
    pub layer: i32,
    pub rect: Rect,
}

/// One box of a special wire.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SBox {
    Wire { layer: i32, rect: Rect },
    Via { rect: Rect, xy: (i32, i32), top: i32, bottom: i32, shapes: Vec<ViaShape> },
}

/// One geometry of a pin (an instance pin already moved by its instance's transform).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Geom {
    pub is_via: bool,
    /// The box's own tech layer (none for a via).
    pub layer: Option<i32>,
    pub rect: Rect,
    pub via_shapes: Vec<ViaShape>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ITerm {
    pub id: i32,
    pub name: String,
    pub placed: bool,
    pub fixed: bool,
    pub bbox_inverted: bool,
    pub avg: (i32, i32),
    pub bridge: bool,
    /// Every geometry of every pin of the terminal, in order.
    pub geoms: Vec<Geom>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BPinBox {
    pub geom: Geom,
    pub disconnect: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BPin {
    pub id: i32,
    /// Any placed status (placed, fixed, ...).
    pub placed: bool,
    pub disconnect: bool,
    pub boxes: Vec<BPinBox>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BTerm {
    pub name: String,
    pub pins: Vec<BPin>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetInput {
    pub name: String,
    pub supply: bool,
    pub swires: Vec<Vec<SBox>>,
    pub iterms: Vec<ITerm>,
    pub bterms: Vec<BTerm>,
}

/// A cursor over a flat stream; running off the end is a malformed stream, never a panic.
struct Stream<'a> {
    v: &'a [i64],
    at: usize,
}

impl Stream<'_> {
    fn next(&mut self) -> Result<i64, String> {
        let x = *self.v.get(self.at).ok_or_else(|| format!("stream ends at {}", self.at))?;
        self.at += 1;
        Ok(x)
    }
    fn int(&mut self) -> Result<i32, String> {
        let x = self.next()?;
        i32::try_from(x).map_err(|_| format!("value {x} out of range at {}", self.at - 1))
    }
    fn count(&mut self) -> Result<usize, String> {
        let x = self.next()?;
        usize::try_from(x).map_err(|_| format!("count {x} at {}", self.at - 1))
    }
    fn flag(&mut self) -> Result<bool, String> {
        Ok(self.next()? != 0)
    }
    fn layer(&mut self) -> Result<Option<i32>, String> {
        let x = self.int()?;
        Ok((x >= 0).then_some(x))
    }
    fn rect(&mut self) -> Result<Rect, String> {
        Ok(Rect { xl: self.int()?, yl: self.int()?, xh: self.int()?, yh: self.int()? })
    }
    fn via_shapes(&mut self) -> Result<Vec<ViaShape>, String> {
        let n = self.count()?;
        (0..n).map(|_| Ok(ViaShape { layer: self.int()?, rect: self.rect()? })).collect()
    }
    fn geom(&mut self) -> Result<Geom, String> {
        let is_via = self.flag()?;
        let layer = self.layer()?;
        let rect = self.rect()?;
        Ok(Geom { is_via, layer, rect, via_shapes: self.via_shapes()? })
    }
    fn done(&self) -> Result<(), String> {
        if self.at == self.v.len() {
            Ok(())
        } else {
            Err(format!("{} values left over", self.v.len() - self.at))
        }
    }
}

/// The tech table from `psm_tech_layers`; `names` are the layer names by number.
pub fn decode_tech(v: &[i64], name_of: impl Fn(i32) -> String) -> Result<Tech, String> {
    let mut s = Stream { v, at: 0 };
    let dbus = s.int()?;
    let front_base = s.layer()?;
    let back_base = s.layer()?;
    let n = s.count()?;
    let mut layers = Vec::with_capacity(n);
    for _ in 0..n {
        let id = u32::try_from(s.next()?).map_err(|e| e.to_string())?;
        let number = s.int()?;
        let routing_level = s.int()?;
        let pitch = s.int()?;
        let dir = match s.next()? {
            1 => Dir::Horizontal,
            2 => Dir::Vertical,
            _ => Dir::None,
        };
        let backside = s.flag()?;
        let upper = s.layer()?;
        let rules = s.count()?;
        let cut_classes = (0..rules)
            .map(|_| {
                Ok(CutClass {
                    cuts_valid: s.flag()?,
                    width: s.int()?,
                    length_valid: s.flag()?,
                    length: s.int()?,
                    num_cuts: s.int()?,
                })
            })
            .collect::<Result<_, String>>()?;
        layers.push(Layer { id, number, name: name_of(number), routing_level, pitch, dir, backside, upper, cut_classes });
    }
    s.done()?;
    Ok(Tech { dbus, front_base, back_base, layers })
}

/// One net from `psm_net`; `names` are its iterm names then its bterm names (`psm_net_names`).
pub fn decode_net(name: &str, v: &[i64], names: &[String]) -> Result<NetInput, String> {
    let mut s = Stream { v, at: 0 };
    let supply = s.flag()?;
    let nwires = s.count()?;
    let mut swires = Vec::with_capacity(nwires);
    for _ in 0..nwires {
        let nboxes = s.count()?;
        let mut boxes = Vec::with_capacity(nboxes);
        for _ in 0..nboxes {
            if s.flag()? {
                let rect = s.rect()?;
                let xy = (s.int()?, s.int()?);
                let top = s.int()?;
                let bottom = s.int()?;
                boxes.push(SBox::Via { rect, xy, top, bottom, shapes: s.via_shapes()? });
            } else {
                boxes.push(SBox::Wire { layer: s.int()?, rect: s.rect()? });
            }
        }
        swires.push(boxes);
    }
    let nterms = s.count()?;
    let mut names = names.iter();
    let mut iterms = Vec::with_capacity(nterms);
    for _ in 0..nterms {
        let id = s.int()?;
        let placed = s.flag()?;
        let fixed = s.flag()?;
        let bbox_inverted = s.flag()?;
        let _avg_ok = s.flag()?;
        let avg = (s.int()?, s.int()?);
        let bridge = s.flag()?;
        let ngeoms = s.count()?;
        let geoms = (0..ngeoms).map(|_| s.geom()).collect::<Result<_, String>>()?;
        let name = names.next().cloned().unwrap_or_default();
        iterms.push(ITerm { id, name, placed, fixed, bbox_inverted, avg, bridge, geoms });
    }
    let nbterms = s.count()?;
    let mut bterms = Vec::with_capacity(nbterms);
    for _ in 0..nbterms {
        let npins = s.count()?;
        let mut pins = Vec::with_capacity(npins);
        for _ in 0..npins {
            let id = s.int()?;
            let placed = s.flag()?;
            let disconnect = s.flag()?;
            let nboxes = s.count()?;
            let mut boxes = Vec::with_capacity(nboxes);
            for _ in 0..nboxes {
                let is_via = s.flag()?;
                let layer = s.layer()?;
                let rect = s.rect()?;
                let disconnect = s.flag()?;
                let via_shapes = s.via_shapes()?;
                boxes.push(BPinBox { geom: Geom { is_via, layer, rect, via_shapes }, disconnect });
            }
            pins.push(BPin { id, placed, disconnect, boxes });
        }
        let name = names.next().cloned().unwrap_or_default();
        bterms.push(BTerm { name, pins });
    }
    s.done()?;
    Ok(NetInput { name: name.to_string(), supply, swires, iterms, bterms })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_a_net_in_stream_order() {
        // One swire with a wire and a two-shape via; one iterm with one plain geometry; one
        // bterm with one pin of one box.
        let v: Vec<i64> = vec![
            1, 1, 2, //
            0, 3, 0, 0, 100, 10, //
            1, 40, 0, 60, 10, 50, 5, 5, 3, 2, 4, 40, 0, 60, 10, 3, 45, 0, 55, 10, //
            1, 7, 1, 1, 0, 1, 20, 5, 0, 1, 0, 3, 10, 0, 30, 10, 0, //
            1, 1, 9, 1, 1, 1, 0, 5, 0, 0, 10, 10, 0, 0,
        ];
        let names = vec!["u1/VDD".to_string(), "VDD".to_string()];
        let n = decode_net("VDD", &v, &names).unwrap();
        assert!(n.supply);
        assert_eq!(n.swires[0].len(), 2);
        assert!(matches!(n.swires[0][1], SBox::Via { xy: (50, 5), top: 5, bottom: 3, ref shapes, .. } if shapes.len() == 2));
        assert_eq!(n.iterms[0].name, "u1/VDD");
        assert_eq!(n.iterms[0].geoms[0].layer, Some(3));
        assert_eq!(n.bterms[0].pins[0].id, 9);
        assert!(n.bterms[0].pins[0].disconnect);
        assert_eq!(n.bterms[0].name, "VDD");
        // A stream with a value left over is malformed.
        let mut long = v.clone();
        long.push(0);
        assert!(decode_net("VDD", &long, &names).is_err());
        assert!(decode_net("VDD", &v[..v.len() - 1], &names).is_err());
    }
}
