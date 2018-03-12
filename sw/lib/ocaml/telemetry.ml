(*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *)

(**
 * Periodic telemetry module for parsing XML config files
 *)

let find_opt = fun k l -> try Some (List.assoc k l) with Not_found -> None
let find_opt_int = fun k l -> try Some (int_of_string (List.assoc k l)) with Not_found -> None

type msg_period = MsgPeriod of float | MsgFreq of float

module Message = struct

  type t = {
    name: string;
    period: msg_period;
    phase: int option;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("message", attribs, []) as xml ->
        let test attrib = List.mem_assoc attrib attribs in
        let fget attrib = float_of_string (List.assoc attrib attribs) in
        {
          name = List.assoc "message" attribs;
          period = begin
            match test "period", test "freq" with
            | true, false -> MsgPeriod (fget "period")
            | false, true -> MsgFreq (fget "period")
            | true, true -> failwith "Telemetry.Message.from_xml: either specify 'period' or 'freq' attribute, not both"
            | false, false -> failwith "Telemetry.Message.from_xml: specify 'period' or 'freq' attribute"
          end;
          phase = find_opt_int "phase" attribs;
          xml
        }
    | _ -> failwith "Telemetry.Message.from_xml: unreachable"

end

module Mode = struct

  type t = {
    name: string;
    key_press: string option;
    messages: Message.t list;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("mode", attribs, messages) as xml ->
        {
          name = List.assoc "name" attribs;
          key_press = find_opt "key_press" attribs;
          messages = List.map Message.from_xml messages;
          xml
        }
    | _ -> failwith "Telemetry.Mode.from_xml: unreachable"

end

module Process = struct

  type t = {
    name: string;
    proc_type: string option;
    modes: Mode.t list;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("process", attribs, modes) as xml ->
        {
          name = List.assoc "name" attribs;
          proc_type = find_opt "type" attribs;
          modes = List.map Mode.from_xml modes;
          xml
        }
    | _ -> failwith "Telemetry.Process.from_xml: unreachable"

end

type t = {
  filename: string;
  processes: Process.t list;
  xml: Xml.xml
}

let from_xml = function
  | Xml.Element ("telemetry", [], processes) as xml ->
      { filename = "";
        processes = List.map Process.from_xml processes;
        xml
      }
  | _ -> failwith "Telemetry.from_xml: unreachable"

let from_file = fun filename ->
  let t = from_xml (Xml.parse_file filename) in
  { t with filename }
